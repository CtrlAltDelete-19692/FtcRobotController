package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

public class Drive {
    private Hardware hw;

    public enum DriveMode { FIELD_CENTRIC, MANUAL }
    DriveMode driveMode = DriveMode.FIELD_CENTRIC;
    
    private static final double BASE_SPEED_LIMIT = 0.6;
    private static final double SLOW_MODE_FACTOR = 0.5;   // right bumper
    private static final double TURBO_EXTRA_FACTOR = 0.4;   // left trigger
    private static final double STICK_DEADZONE = 0.05;
    private static final double STRAFE_CORRECTION = 1.1;

    double LFM = 0;
    double LBM = 0;
    double RFM = 0;
    double RBM = 0;
    
    private boolean psPressedLast = false;
    private boolean selectPressedLast = false;

    public void setup(Hardware hardware) {
        hw = hardware;
    }

    public void update(Gamepad gamepad) {
        if (hw == null) {
            throw new IllegalStateException("Drive.setup(Hardware) must be called before update().");
        }
    
        // Reset heading - center Logitech button is ps button
        boolean psPressed = gamepad.ps;
        if (psPressed && !psPressedLast) {
            hw.imu.resetYaw();
        }
        psPressedLast = psPressed;


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
        if (gamepad.right_bumper) {
            speedLimit = BASE_SPEED_LIMIT * SLOW_MODE_FACTOR;
        } else if (gamepad.left_bumper) {
            speedLimit = Range.clip(BASE_SPEED_LIMIT + gamepad.left_trigger * TURBO_EXTRA_FACTOR, 0.0, 1.0);
        }
        
        double lx = gamepad.left_stick_x;
        double ly = -gamepad.left_stick_y; // Remember, Y stick value is reversed
        double rx = gamepad.right_stick_x;
        
        // Create small deadzone to accommodate janky / old controllers
        if (Math.abs(lx) < STICK_DEADZONE) lx = 0;
        if (Math.abs(ly) < STICK_DEADZONE) ly = 0;
        if (Math.abs(rx) < STICK_DEADZONE) rx = 0;

        driveWithMode(lx, ly, rx, speedLimit);
    }
    
    private void driveWithMode(double strafe, double forward, double rotate, double speedLimit) {
        if (driveMode == DriveMode.FIELD_CENTRIC) {
            double headingRad = -hw.imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS);
    
            double tempX = strafe * Math.cos(headingRad) - forward * Math.sin(headingRad);
            double tempY = strafe * Math.sin(headingRad) + forward * Math.cos(headingRad);
    
            // Counteract imperfect strafing
            strafe = tempX * STRAFE_CORRECTION;
            forward = tempY;
        }
    
        drive(strafe, forward, rotate, speedLimit);
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

        hw.leftFrontMotor.setPower(LFM);
        hw.rightFrontMotor.setPower(RFM);
        hw.leftBackMotor.setPower(LBM);
        hw.rightBackMotor.setPower(RBM);
    }

    public void centerOnTag(int teamTagId) {
        LLResult result = hw.limelight.getLatestResult();
        double rotate = 26;
        if (result != null && result.isValid()) {
            double tx = result.getTx();
            if (Math.abs(tx) < 1) {
                return;
            }

            rotate = tx;
        }

        rotate = 0.01 * rotate;
        drive(0, 0, rotate, 1);
    }

    
    public DriveMode getDriveMode() {
        return driveMode;
    }
    
    public double[] getWheelPowers() {
        return new double[] { LFM, RFM, LBM, RBM };
    }
}
