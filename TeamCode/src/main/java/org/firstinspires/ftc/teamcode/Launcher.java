package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Gamepad;

public class Launcher {
    private final Hardware hw;

    private static final double LOADER_POWER = 0.5; // Between 0 and 1
    private static final double LAUNCHER_IDLE_TICKS = 0;  // maintains spin
    public static final double LAUNCHER_BASE_TICKS = 1050;  // full shooting speed from 2ft
    public static final double TICKS_PER_FOOT = 60;  // ticks to add for each extra foot of distance, based on testing
    public static final int LAUNCHER_TICKS_INCREMENTS = 10;  // When manually adjusting launcher, increment / decrement by this amount

    public double launcherVelocity = 0;
    public int lvManualAdjustment = 0;
    public int lvGoalDistanceAdjustment = 0;

    public Launcher(Hardware hardware) {
        this.hw = hardware;
    }

    public void update(Gamepad gamepad, Gamepad gamepad2) {
        // Do not move the loader or launcher motors unless we are on the ground!
        boolean onGround = hw.leftViperSlideMotor.getCurrentPosition() <= 30 && hw.rightViperSlideMotor.getCurrentPosition() <= 30;
        if (! onGround) {
            return;
        }

        loader(gamepad, gamepad2);
        launcher(gamepad, gamepad2);
    }

    private void loader(Gamepad gamepad, Gamepad gamepad2) {
        if (hw.killMotors) {
            hw.loader.setPower(0);
            return;
        }

        if (hw.loader != null) {
            boolean launcherReady = upToSpeed(); // Within 94% of target
            boolean oneController = gamepad2.a && DecodeTeleOp.oneController;
            boolean oneControllerB = gamepad2.b && DecodeTeleOp.oneController;
            if (((gamepad.a || oneController) && launcherReady) || (gamepad.b || oneControllerB)) {
            //if (gamepad.a || oneController) {
                hw.loader.setPower(LOADER_POWER);
            } else {
                hw.loader.setPower(0);
            }
        }
    }

    private void launcher(Gamepad gamepad, Gamepad gamepad2) {
        if (hw.killMotors) {
            hw.launcher.setVelocity(0);
            return;
        }

        launcherVelocity = LAUNCHER_IDLE_TICKS;

        // Manual launch velocity adjustment
        if (gamepad.left_bumper) {
            lvManualAdjustment -= LAUNCHER_TICKS_INCREMENTS;
        } else if(gamepad.right_bumper) {
            lvManualAdjustment += LAUNCHER_TICKS_INCREMENTS;
        }

        lvGoalDistanceAdjustment = getGoalDistanceAdjustment();

        // Set launch velocity
        boolean oneController = gamepad2.right_trigger > Hardware.TRIGGER_DEADZONE && DecodeTeleOp.oneController;
        if (gamepad.right_trigger > Hardware.TRIGGER_DEADZONE || oneController) {
            launcherVelocity = LAUNCHER_BASE_TICKS + lvManualAdjustment + lvGoalDistanceAdjustment;
        } else if (gamepad.left_trigger > Hardware.TRIGGER_DEADZONE) {
            launcherVelocity = (LAUNCHER_BASE_TICKS + lvManualAdjustment + lvGoalDistanceAdjustment) * -1;
        }

        hw.launcher.setVelocity(launcherVelocity);
    }

    public void stopLauncher() {
        hw.launcher.setVelocity(0);
    }

    public void autoLauncher(int ticksPerSecond) {
        int tagAdjustment = getGoalDistanceAdjustment();
        if (tagAdjustment == -1) {
            tagAdjustment = ticksPerSecond;
        }
        launcherVelocity = LAUNCHER_BASE_TICKS + tagAdjustment;
        hw.launcher.setVelocity(launcherVelocity);
    }

    // Automatic launch velocity adjustment based on tag distance
    public int getGoalDistanceAdjustment() {
        int adjustment = -1;

        if (hw.aprilTag.tagSeen && !Double.isNaN(hw.aprilTag.z)) {
            double feet = hw.aprilTag.z * 3.28 - 1.5; // There are 3.28ft in a meter, and z is in meters. LAUNCHER_BASE_TICKS is tuned to 2ft, so we subtract 2.
            if (feet > 0) {
                adjustment = (int) (feet * TICKS_PER_FOOT);
                adjustment = adjustment - (adjustment % 10); // Round to the 10 so slight variations in tag reads don't constantly toggle velocity
            }
        }

        return adjustment;
    }

    public boolean upToSpeed() {
        if (hw.launcher.getVelocity() >= (launcherVelocity * 0.94)) {
            return true;
        }
        return false;
    }
}
