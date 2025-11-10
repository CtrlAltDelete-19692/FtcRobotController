package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Gamepad;

public class Launcher {
    private final Hardware hw;

    private static final double LOADER_POWER = 0.5; // Between 0 and 1
    private static final double LAUNCHER_IDLE_RPM = 0;  // maintains spin
    private static final double LAUNCHER_FULL_RPM = 200;  // full shooting speed
    private static final double TRIGGER_DEADZONE = 0.05;

    public double launcherVelocity = 0;
    public double x = -1;

    public double z = -1;

    public Launcher(Hardware hardware) {
        this.hw = hardware;
    }

    public void update(Gamepad gamepad) {
        // Do not move the loader or launcher motors unless we are on the ground!
        if (hw.leftViperSlideMotor.getCurrentPosition() > 30 || hw.rightViperSlideMotor.getCurrentPosition() > 30) {
            return;
        }

        loader(gamepad);
        launcher(gamepad);
    }

    private void loader(Gamepad gamepad) {
        if (hw.loader != null) {
            double currentLauncherVelocity = hw.launcher.getVelocity();
            boolean launcherReady = currentLauncherVelocity >= (LAUNCHER_FULL_RPM * 0.95); // Within 95% of FULL_RPM
            //if ((gamepad.right_bumper || gamepad.a) && launcherReady) {
            if (gamepad.right_bumper || gamepad.a) {
                hw.loader.setPower(LOADER_POWER);
            } else {
                hw.loader.setPower(0);
            }
        }
    }

    private void launcher(Gamepad gamepad) {
        z = 3; // TODO: Update this default accordingly
        x = 0;

        AprilTag aprilTag = null;
        if (hw.limelight != null) {
            aprilTag = new AprilTag(hw.limelight);
        }
        if (aprilTag != null) {
            z = aprilTag.getZ();
            x = aprilTag.getX();
        }
        launcherVelocity = LAUNCHER_IDLE_RPM;
        if (gamepad.right_trigger > TRIGGER_DEADZONE) {
            launcherVelocity = LAUNCHER_FULL_RPM;
            if (z <= 0.3) { // TODO: Tune accordingly, maybe set to linear relationship
                launcherVelocity *= 0.5;
            } else if (z <= 0.6) {
                launcherVelocity *= 0.7;
            }
        }
        hw.launcher.setVelocity(launcherVelocity);
    }
}
