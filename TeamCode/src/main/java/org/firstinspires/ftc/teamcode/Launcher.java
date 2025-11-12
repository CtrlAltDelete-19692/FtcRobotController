package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Gamepad;

public class Launcher {
    private final Hardware hw;

    private static final double LOADER_POWER = 0.5; // Between 0 and 1
    private static final double LAUNCHER_IDLE_TICKS = 0;  // maintains spin
    private static final double LAUNCHER_FULL_TICKS = 500;  // full shooting speed
    private static final double TRIGGER_DEADZONE = 0.05;

    public double launcherVelocity = 0;

    public int add = 0;

    public Launcher(Hardware hardware) {
        this.hw = hardware;
    }

    public void update(Gamepad gamepad) {
        // Do not move the loader or launcher motors unless we are on the ground!
        boolean onGround = hw.leftViperSlideMotor.getCurrentPosition() <= 30 && hw.rightViperSlideMotor.getCurrentPosition() <= 30;
        if (! onGround) {
            return;
        }

        loader(gamepad);
        launcher(gamepad);
    }

    private void loader(Gamepad gamepad) {
        if (hw.loader != null) {
            double currentLauncherVelocity = hw.launcher.getVelocity();
            boolean launcherReady = currentLauncherVelocity >= (LAUNCHER_FULL_TICKS * 0.95); // Within 95% of FULL_RPM
            //if ((gamepad.right_bumper || gamepad.a) && launcherReady) {
            if (gamepad.right_bumper || gamepad.a) {
                hw.loader.setPower(LOADER_POWER);
            } else {
                hw.loader.setPower(0);
            }
        }
    }

    private void launcher(Gamepad gamepad) {
        launcherVelocity = LAUNCHER_IDLE_TICKS;
        if (gamepad.right_trigger > TRIGGER_DEADZONE) {
            launcherVelocity = LAUNCHER_FULL_TICKS + add;
            if (hw.aprilTag.tagSeen && !Double.isNaN(hw.aprilTag.z)) {
                if (hw.aprilTag.z <= 0.3)      launcherVelocity *= 0.5; // TODO: Tune accordingly, maybe set to linear relationship
                else if (hw.aprilTag.z <= 0.6) launcherVelocity *= 0.7;
            }
        }

        if (gamepad.left_trigger > TRIGGER_DEADZONE && gamepad.x) {
            add += 20;
        } else if(gamepad.left_trigger > TRIGGER_DEADZONE && gamepad.y) {
            add -= 20;
        }
        hw.launcher.setVelocity(launcherVelocity);

    }
}
