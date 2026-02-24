package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;

public class Launcher {
    private final Hardware hw;
    public CRServo loader;

    public DcMotorEx launcher;

    private static final double LOADER_POWER = 0.5; // Between 0 and 1
    private static final double LAUNCHER_IDLE_TICKS = 500;  // maintains spin
    public static final double LAUNCHER_BASE_TICKS = 1050;  // full shooting speed from 2ft
    public static final double TICKS_PER_FOOT = 60;  // ticks to add for each extra foot of distance, based on testing
    public static final int LAUNCHER_TICKS_INCREMENTS = 10;  // When manually adjusting launcher, increment / decrement by this amount
    public static final double LAUNCHER_THRESHOLD = 0.94;  // What percentage of launcher set velocity is required to shoot

    public double launcherVelocity = 0;
    public int lvManualAdjustment = 0;
    public int lvGoalDistanceAdjustment = 0;
    public boolean readyToLaunch = false;

    public Launcher(Hardware hardware, HardwareMap hardwareMap) {
        this.hw = hardware;
        setup(hardwareMap);
    }

    public void setup(HardwareMap hardwareMap) {
        loader = hardwareMap.get(CRServo.class, "Loader");
        if (loader != null) {
            loader.setDirection(CRServo.Direction.REVERSE);
            loader.setPower(0);
        }

        launcher = hardwareMap.get(DcMotorEx.class, "Launcher");
        launcher.setDirection(DcMotor.Direction.REVERSE);
        launcher.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        launcher.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        launcher.setVelocity(0);
        PIDFCoefficients newCoeffs = new PIDFCoefficients(14  , 0, 0.8, 13.2);
        launcher.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, newCoeffs);
    }

    public void update(Gamepad gamepad, Gamepad gamepad2) {
        // Do not move the loader or launcher motors unless we are on the ground!
        //boolean onGround = hw.leftViperSlideMotor.getCurrentPosition() <= 30 && hw.rightViperSlideMotor.getCurrentPosition() <= 30;
        //if (! onGround) {
        //    return;
        //}

        launcher(gamepad, gamepad2);
        loader(gamepad, gamepad2);
    }

    private void loader(Gamepad gamepad, Gamepad gamepad2) {
        if (hw.killMotors) {
            loader.setPower(0);
            return;
        }

        if (loader != null) {
            boolean oneController = gamepad2.a && DecodeTeleOp.oneController;
            boolean oneControllerB = gamepad2.b && DecodeTeleOp.oneController;
            if (((gamepad.a || oneController) && readyToLaunch) || (gamepad.b || oneControllerB)) {
                loader.setPower(LOADER_POWER);
            } else {
                loader.setPower(0);
            }
        }
    }

    private void launcher(Gamepad gamepad, Gamepad gamepad2) {
        boolean oneController = gamepad2.right_trigger > Hardware.TRIGGER_DEADZONE && DecodeTeleOp.oneController;
        boolean spinLauncher = gamepad.right_trigger > Hardware.TRIGGER_DEADZONE || oneController;

        if (spinLauncher && upToSpeed()) {
            readyToLaunch = true;
        } else {
            readyToLaunch = false;
        }

        //if (hw.leftViperSlideMotor.getCurrentPosition() > 30 || hw.rightViperSlideMotor.getCurrentPosition() > 30 || hw.killMotors) {
        if (hw.killMotors) {
            launcher.setVelocity(0);
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
        if (spinLauncher) {
            launcherVelocity = LAUNCHER_BASE_TICKS + lvManualAdjustment + lvGoalDistanceAdjustment;
        } else if (gamepad.left_trigger > Hardware.TRIGGER_DEADZONE) {
            launcherVelocity = (LAUNCHER_BASE_TICKS + lvManualAdjustment + lvGoalDistanceAdjustment) * -1;
        }

        launcher.setVelocity(launcherVelocity);
    }

    public void stopLauncher() {
        launcher.setVelocity(0);
    }

    public void autoLauncher(int ticksPerSecond) {
        int tagAdjustment = getGoalDistanceAdjustment();
        if (tagAdjustment == -1) {
            tagAdjustment = ticksPerSecond;
        }
        launcherVelocity = LAUNCHER_BASE_TICKS + tagAdjustment;
        launcher.setVelocity(launcherVelocity);
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
        return launcher.getVelocity() >= (launcherVelocity * LAUNCHER_THRESHOLD);
    }
}
