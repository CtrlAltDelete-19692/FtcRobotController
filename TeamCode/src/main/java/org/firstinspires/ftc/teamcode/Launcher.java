package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Gamepad;

public class Launcher {
    private final Hardware hw;

    private static final double LOADER_POWER = 0.5; // Between 0 and 1
    private static final double LAUNCHER_IDLE_TICKS = 150;  // maintains spin
    public static final double LAUNCHER_FULL_TICKS = 480;  // full shooting speed
    public static final int LAUNCHER_TICKS_INCREMENTS = 10;  // When manually adjusting launcher, increment / decrement by this amount
    public static final int LAUNCHER_THREE_POINTER_ADDITIONAL_TICKS = 60;  // How many addition launcher ticks to add from the small white triangle

    public double launcherVelocity = 0;
    public int lvManualAdjustment = 0;
    public int lvGoalDistanceAdjustment = 0;

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
            //if (gamepad.a && launcherReady) {
            if (gamepad.a) {
                hw.loader.setPower(LOADER_POWER);
            } else {
                hw.loader.setPower(0);
            }
        }
    }

    private void launcher(Gamepad gamepad) {
        launcherVelocity = LAUNCHER_IDLE_TICKS;

        // Automatic launch velocity adjustment based on tag distance
        if (hw.aprilTag.tagSeen && !Double.isNaN(hw.aprilTag.z)) {
                /*if (hw.aprilTag.z <= 0.3)      lvGoalDistanceAdjustment = -60; // TODO: Tune accordingly, maybe set to linear relationship
                else if (hw.aprilTag.z <= 0.6) lvGoalDistanceAdjustment = -40;*/
            if (hw.aprilTag.z <= 1) { // In close white triangle, scale down according to closeness to team tag
                lvGoalDistanceAdjustment = (int)(hw.aprilTag.z * 70);
            } else if (hw.aprilTag.z > 2) { // Far white triangle
                lvGoalDistanceAdjustment = LAUNCHER_THREE_POINTER_ADDITIONAL_TICKS;
            }
        } else {
            lvGoalDistanceAdjustment = 0;
        }

        // Manual launch velocity adjustment
        if (gamepad.left_bumper) {
            lvManualAdjustment -= LAUNCHER_TICKS_INCREMENTS;
        } else if(gamepad.right_bumper) {
            lvManualAdjustment += LAUNCHER_TICKS_INCREMENTS;
        }

        // Set launch velocity
        if (gamepad.right_trigger > Hardware.TRIGGER_DEADZONE) {
            launcherVelocity = LAUNCHER_FULL_TICKS + lvManualAdjustment + lvGoalDistanceAdjustment;
        }

        hw.launcher.setVelocity(launcherVelocity);
    }
}
