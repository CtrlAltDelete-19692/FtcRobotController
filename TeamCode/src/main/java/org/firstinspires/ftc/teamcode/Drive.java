package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

public class Drive {
    private Hardware hw;

    public DcMotor rightFrontMotor;
    public DcMotor rightBackMotor;
    public DcMotor leftFrontMotor;
    public DcMotor leftBackMotor;

    public enum DriveMode { FIELD_CENTRIC, MANUAL }
    DriveMode driveMode = DriveMode.FIELD_CENTRIC; // Default drive mode
    
    private static final double BASE_SPEED_LIMIT = 0.75;
    private static final double AUTO_SPEED = 0.6;
    private static final double SLOW_MODE_FACTOR = 0.2;
    private static final double TURBO_EXTRA_FACTOR = 0.15;
    private static final double STRAFE_CORRECTION = 1.7;

    double LFM = 0;
    double LBM = 0;
    double RFM = 0;
    double RBM = 0;

    private boolean selectPressedLast = false;

    public Drive(Hardware hw, HardwareMap hardwareMap, String driveSystem) {
        this.hw = hw;

        setup(hardwareMap, driveSystem);
    }

    public void setup(HardwareMap hardwareMap, String driveSystem) {
        leftFrontMotor = hardwareMap.get(DcMotor.class, "LFM");
        DcMotor.Direction left = DcMotor.Direction.REVERSE;
        if (driveSystem.equals("King Bob")) {
            left = DcMotor.Direction.FORWARD;
        }
        leftFrontMotor.setDirection(left);
        leftFrontMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        leftBackMotor  = hardwareMap.get(DcMotor.class, "LBM");
        leftBackMotor.setDirection(left);
        leftBackMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        rightFrontMotor = hardwareMap.get(DcMotor.class, "RFM");
        rightFrontMotor.setDirection(DcMotor.Direction.FORWARD);
        rightFrontMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        rightBackMotor = hardwareMap.get(DcMotor.class, "RBM");
        rightBackMotor.setDirection(DcMotor.Direction.FORWARD);
        rightBackMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }

    public void update(Gamepad gamepad, Gamepad gamepad2, int teamTagId) {
        if (hw == null) {
            throw new IllegalStateException("Hardware not found during drive.update().");
        }

        driveSystem(gamepad, gamepad2, teamTagId);
    }
    
    private void driveSystem(Gamepad gamepad, Gamepad gamepad2, int teamTagId) {
        // Drive Mode
        boolean selectPressed = gamepad.back || gamepad.share;
        if (selectPressed && !selectPressedLast) {
            if (driveMode == DriveMode.FIELD_CENTRIC) {
                driveMode = DriveMode.MANUAL;
            } else {
                driveMode = DriveMode.FIELD_CENTRIC;
            }
        }
        selectPressedLast = selectPressed;

        // Speed modes
        double speedLimit = BASE_SPEED_LIMIT;
        if (gamepad.left_trigger > Hardware.TRIGGER_DEADZONE) {
            speedLimit = BASE_SPEED_LIMIT * SLOW_MODE_FACTOR;
        } else if (gamepad.right_trigger > Hardware.TRIGGER_DEADZONE) {
            speedLimit = Range.clip(BASE_SPEED_LIMIT + gamepad.right_trigger * TURBO_EXTRA_FACTOR, 0.0, 1.0);
        }

        double strafe = gamepad.left_stick_x;
        double forward = -gamepad.left_stick_y; // Remember, Y stick value is reversed
        double rotate = gamepad.right_stick_x;

        // Create small deadzone to accommodate janky / old controllers
        if (Math.abs(strafe) < Hardware.STICK_DEADZONE) strafe = 0;
        if (Math.abs(forward) < Hardware.STICK_DEADZONE) forward = 0;
        if (Math.abs(rotate) < Hardware.STICK_DEADZONE) rotate = 0;

        if (driveMode == DriveMode.FIELD_CENTRIC) {
            double headingRad = -hw.imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS);
    
            double tempX = strafe * Math.cos(headingRad) - forward * Math.sin(headingRad);
            double tempY = strafe * Math.sin(headingRad) + forward * Math.cos(headingRad);

            strafe = tempX;
            forward = tempY;
        }

        strafe = strafe * STRAFE_CORRECTION; // Counteract imperfect strafing

        boolean oneController = gamepad.right_trigger > Hardware.TRIGGER_DEADZONE && DecodeTeleOp.oneController;
        if (gamepad2.right_trigger > Hardware.TRIGGER_DEADZONE || oneController) {
            rotate = getAutoRotate(rotate, teamTagId);
        }

        drive(strafe, forward, rotate, speedLimit);
    }

    public double getAutoRotate(double rotate, int teamTagId) {
        LLResult result = hw.limelight.getLatestResult();
        //double rotate = AUTO_AIM_SPEED;
        if (result != null && result.isValid()) {
            double tx = result.getTx();
            //if (teamTagId == 20) { // Blue
            //    tx = tx - 8;
            //}

            if (Math.abs(tx) < 1.0) {
                return 0;
            }

            rotate = 0.02 * tx;
        }

        return rotate;
    }

    private void drive(double strafe, double forward, double rotate, double speedLimit) {
        LFM = forward + strafe + rotate;
        RFM = forward - strafe - rotate;
        LBM = forward - strafe + rotate;
        RBM = forward + strafe - rotate;

        // Normalize so no value exceeds 1.0
        double max = Math.max(1.0, Math.max(Math.abs(LFM), Math.max(Math.abs(RFM), Math.max(Math.abs(LBM), Math.abs(RBM)))));

        LFM = Range.clip(LFM / max, -speedLimit, speedLimit);
        RFM = Range.clip(RFM / max, -speedLimit, speedLimit);
        LBM = Range.clip(LBM / max, -speedLimit, speedLimit);
        RBM = Range.clip(RBM / max, -speedLimit, speedLimit);

        // Do not allow the wheels to move unless we are on the ground or while we kill the motors
        //if (hw.leftViperSlideMotor.getCurrentPosition() > 30 || hw.rightViperSlideMotor.getCurrentPosition() > 30 || hw.killMotors) {
        if (hw.killMotors) {
            LFM = 0;
            RFM = 0;
            LBM = 0;
            RBM = 0;
        }

        leftFrontMotor.setPower(LFM);
        rightFrontMotor.setPower(RFM);
        leftBackMotor.setPower(LBM);
        rightBackMotor.setPower(RBM);
    }

    public void driveCommand(double strafe, double forward, double rotate) {
        drive(strafe, forward, rotate, AUTO_SPEED);
    }

    public DriveMode getDriveMode() {
        return driveMode;
    }
    
    public double[] getWheelPowers() {
        return new double[] { LFM, RFM, LBM, RBM };
    }
}
