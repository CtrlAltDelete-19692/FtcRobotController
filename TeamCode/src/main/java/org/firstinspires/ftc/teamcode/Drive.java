package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.VoltageSensor;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

public class Drive {
    public DcMotor rightFrontMotor;
    public DcMotor rightBackMotor;
    public DcMotor leftFrontMotor;
    public DcMotor leftBackMotor;

    public IMU imu;
    public IMU.Parameters imuParams;

    public VoltageSensor voltageSensor;
    public Limelight3A limelight;

    public enum DriveMode { FIELD_CENTRIC, MANUAL }
    DriveMode driveMode = DriveMode.FIELD_CENTRIC; // Default drive mode
    
    private static final double BASE_SPEED_LIMIT = 1;
    private static final double AUTO_SPEED = 1;
    private static final double SLOW_MODE_FACTOR = 0.35;
    private static final double STRAFE_CORRECTION = 1.6;
    private static final double BIOBUZZ_ROTATION_SCALE = 0.85; // Reduce manual chassis rotation by 15%.
    private static final double DRIVE_ACCEL_RATE = 2.0; // Stick input per second.
    private static final double DRIVE_DECEL_RATE = 6.0;
    private static final double MAX_DRIVE_UPDATE_SECONDS = 0.1;

    double LFM = 0;
    double LBM = 0;
    double RFM = 0;
    double RBM = 0;

    private boolean selectPressedLast = false;
    private boolean rbPressedLast = false;
    private boolean resetYawOnRightBumper = true;
    private final boolean chassisAutoAimEnabled;
    private final double manualRotationScale;
    private final boolean smoothManualDrive;
    private long lastDriveUpdateNanos;
    private double smoothStrafe;
    private double smoothForward;
    private double smoothRotate;

    public Drive(HardwareMap hardwareMap, String driveSystem, boolean includeLimelight) {
        // Biobuzz aims its turret independently; RT must leave chassis rotation under driver control.
        chassisAutoAimEnabled = !driveSystem.equals("Biobuzz");
        manualRotationScale = driveSystem.equals("Biobuzz") ? BIOBUZZ_ROTATION_SCALE : 1.0;
        smoothManualDrive = driveSystem.equals("Biobuzz");
        if (includeLimelight) {
            limelight = hardwareMap.get(Limelight3A.class, "Limelight");
        }

        setupMotors(hardwareMap, driveSystem);

        setupImu(hardwareMap, driveSystem);
    }

    public void setupMotors(HardwareMap hardwareMap, String driveSystem) {
        leftFrontMotor = hardwareMap.get(DcMotor.class, "LFM");
        DcMotor.Direction leftFMD = DcMotor.Direction.REVERSE;
        if (driveSystem.equals("Biobuzz")) {
            leftFMD = DcMotor.Direction.REVERSE;
        }
        leftFrontMotor.setDirection(leftFMD);
        leftFrontMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        DcMotor.Direction leftBMD = DcMotor.Direction.FORWARD;
        if (driveSystem.equals("Biobuzz")) {
            leftBMD = DcMotor.Direction.FORWARD;
        }
        leftBackMotor  = hardwareMap.get(DcMotor.class, "LBM");
        leftBackMotor.setDirection(leftBMD);
        leftBackMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        DcMotor.Direction rightFMD = DcMotor.Direction.FORWARD;
        if (driveSystem.equals("Biobuzz")) {
            rightFMD = DcMotor.Direction.REVERSE;
        }
        rightFrontMotor = hardwareMap.get(DcMotor.class, "RFM");
        rightFrontMotor.setDirection(rightFMD);
        rightFrontMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        DcMotor.Direction rightBMD = DcMotor.Direction.FORWARD;
        if (driveSystem.equals("Biobuzz")) {
            rightBMD = DcMotor.Direction.REVERSE;
        }
        rightBackMotor = hardwareMap.get(DcMotor.class, "RBM");
        rightBackMotor.setDirection(rightBMD);
        rightBackMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        if (driveSystem.equals("Biobuzz")) {
            leftFrontMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            leftBackMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            rightFrontMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            rightBackMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        }
    }

    public void setupImu(HardwareMap hardwareMap, String driveSystem) {
        imu = hardwareMap.get(IMU.class, "imu");
        RevHubOrientationOnRobot orientationOnRobot;
        if (driveSystem.equals("Biobuzz")) {
            orientationOnRobot = new RevHubOrientationOnRobot(
                    RevHubOrientationOnRobot.LogoFacingDirection.UP,
                    RevHubOrientationOnRobot.UsbFacingDirection.LEFT
            );
        } else {
            orientationOnRobot = new RevHubOrientationOnRobot(
                    RevHubOrientationOnRobot.LogoFacingDirection.FORWARD,
                    RevHubOrientationOnRobot.UsbFacingDirection.UP
            );
        }
        imuParams = new IMU.Parameters(orientationOnRobot);
        imu.initialize(imuParams);

        voltageSensor = hardwareMap.voltageSensor.iterator().next();
    }

    public void update(Gamepad gamepad, Gamepad gamepad2, int teamTagId) {
        update(gamepad, gamepad2);
    }

    public void update(Gamepad gamepad, Gamepad gamepad2) {
        // Reset heading
        boolean rbPressed = gamepad.right_bumper;
        if (resetYawOnRightBumper && rbPressed && !rbPressedLast) {
            imu.resetYaw();
        }
        rbPressedLast = rbPressed;

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
        if (gamepad.left_trigger > CtrlAltDelOpMode.TRIGGER_DEADZONE) {
            speedLimit = BASE_SPEED_LIMIT * SLOW_MODE_FACTOR;
        }

        double strafe = gamepad.left_stick_x;
        double forward = -gamepad.left_stick_y; // Remember, Y stick value is reversed
        double rotate = gamepad.right_stick_x;

        // Create small deadzone to accommodate janky / old controllers
        if (Math.abs(strafe) < CtrlAltDelOpMode.STICK_DEADZONE) strafe = 0;
        if (Math.abs(forward) < CtrlAltDelOpMode.STICK_DEADZONE) forward = 0;
        if (Math.abs(rotate) < CtrlAltDelOpMode.STICK_DEADZONE) rotate = 0;
        if (manualRotationScale < 1.0) {
            rotate = Range.clip(rotate, -speedLimit, speedLimit) * manualRotationScale;
        }

        if (smoothManualDrive) {
            if (CtrlAltDelOpMode.killMotors) {
                smoothStrafe = smoothForward = smoothRotate = 0;
                lastDriveUpdateNanos = 0;
            } else {
                long now = System.nanoTime();
                double elapsed = lastDriveUpdateNanos == 0 ? 0.02
                        : Math.min(MAX_DRIVE_UPDATE_SECONDS, Math.max(0, (now - lastDriveUpdateNanos) / 1e9));
                lastDriveUpdateNanos = now;
                smoothStrafe = smoothInput(smoothStrafe, strafe, elapsed);
                smoothForward = smoothInput(smoothForward, forward, elapsed);
                smoothRotate = smoothInput(smoothRotate, rotate, elapsed);
            }
            strafe = smoothStrafe;
            forward = smoothForward;
            rotate = smoothRotate;
        }

        if (driveMode == DriveMode.FIELD_CENTRIC) {
            double headingRad = -imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS);
    
            double tempX = strafe * Math.cos(headingRad) - forward * Math.sin(headingRad);
            double tempY = strafe * Math.sin(headingRad) + forward * Math.cos(headingRad);

            strafe = tempX;
            forward = tempY;
        }

        strafe = strafe * STRAFE_CORRECTION; // Counteract imperfect strafing

        boolean oneController = gamepad.right_trigger > CtrlAltDelOpMode.TRIGGER_DEADZONE && DecodeTeleOp.oneController;
        if (chassisAutoAimEnabled
                && (gamepad2.right_trigger > CtrlAltDelOpMode.TRIGGER_DEADZONE || oneController)) {
            rotate = getAutoRotate(rotate);
        }

        drive(strafe, forward, rotate, speedLimit);
    }

    private double smoothInput(double current, double target, double elapsedSeconds) {
        if (current * target < 0) target = 0; // Slow to a stop before reversing.
        double rate = Math.abs(target) > Math.abs(current) ? DRIVE_ACCEL_RATE : DRIVE_DECEL_RATE;
        double maxChange = rate * elapsedSeconds;
        return current + Range.clip(target - current, -maxChange, maxChange);
    }

    public double getAutoRotate(double rotate, int teamTagId) {
        return getAutoRotate(rotate);
    }

    public double getAutoRotate(double rotate) {
        if (limelight == null) {
            return rotate;
        }

        LLResult result = limelight.getLatestResult();
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
        //if (leftViperSlideMotor.getCurrentPosition() > 30 || rightViperSlideMotor.getCurrentPosition() > 30 || CtrlAltDelOpMode.killMotors) {
        if (CtrlAltDelOpMode.killMotors) {
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
        driveCommand(strafe, forward, rotate, 1.0);
    }

    public void driveCommand(double strafe, double forward, double rotate, double speedScale) {
        drive(strafe, forward, rotate, AUTO_SPEED * Range.clip(speedScale, 0.0, 1.0));
    }

    public DriveMode getDriveMode() {
        return driveMode;
    }

    public void setResetYawOnRightBumper(boolean enabled) {
        resetYawOnRightBumper = enabled;
    }

    public void addTelemetry(Telemetry telemetry) {
        double headingDeg = imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES);
        telemetry.addData("Heading", "%.1f deg", headingDeg);
    }

    public void addDebugTelemetry(Telemetry telemetry) {
        addTelemetry(telemetry);
        telemetry.addData("Drive front FL/FR", "%.0f%% / %.0f%%   %d / %d ticks",
                LFM * 100, RFM * 100,
                leftFrontMotor.getCurrentPosition(), rightFrontMotor.getCurrentPosition());
        telemetry.addData("Drive back BL/BR", "%.0f%% / %.0f%%   %d / %d ticks",
                LBM * 100, RBM * 100,
                leftBackMotor.getCurrentPosition(), rightBackMotor.getCurrentPosition());
    }

    public void setDriveMode(DriveMode driveMode) {
        this.driveMode = driveMode;
    }
    
    public double[] getWheelPowers() {
        return new double[] { LFM, RFM, LBM, RBM };
    }

//    public double getBatteryVoltage() {
//        return voltageSensor.getVoltage();
//    }
}
