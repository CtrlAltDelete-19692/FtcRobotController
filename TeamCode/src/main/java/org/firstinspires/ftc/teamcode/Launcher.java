package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;

public class Launcher {
    public AprilTag aprilTag;
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

    public Launcher(HardwareMap hardwareMap, AprilTag aprilTag) {
        setup(hardwareMap);
        this.aprilTag = aprilTag;
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
        //boolean onGround = leftViperSlideMotor.getCurrentPosition() <= 30 && rightViperSlideMotor.getCurrentPosition() <= 30;
        //if (! onGround) {
        //    return;
        //}

        launcher(gamepad, gamepad2);
        loader(gamepad, gamepad2);
    }

    private void loader(Gamepad gamepad, Gamepad gamepad2) {
        if (CtrlAltDelOpMode.killMotors) {
            loader.setPower(0);
            return;
        }

        if (loader != null) {
            boolean oneController = gamepad2.a && DecodeTeleOp.oneController;
            boolean oneControllerB = gamepad2.b && DecodeTeleOp.oneController;
            //if (((gamepad.a || oneController) && readyToLaunch) || (gamepad.b || oneControllerB)) {
            if (gamepad.b) {
                loader.setDirection(CRServo.Direction.REVERSE);
                loader.setPower(LOADER_POWER);
            } else if (gamepad.a) {
                loader.setDirection(CRServo.Direction.FORWARD);
                loader.setPower(LOADER_POWER);
            } else {
                loader.setPower(0);
            }
        }
    }

    private void launcher(Gamepad gamepad, Gamepad gamepad2) {
        boolean oneController = gamepad2.right_trigger > CtrlAltDelOpMode.TRIGGER_DEADZONE && DecodeTeleOp.oneController;
        boolean spinLauncher = gamepad.right_trigger > CtrlAltDelOpMode.TRIGGER_DEADZONE || oneController;

        if (spinLauncher && upToSpeed()) {
            readyToLaunch = true;
        } else {
            readyToLaunch = false;
        }

        //if (leftViperSlideMotor.getCurrentPosition() > 30 || rightViperSlideMotor.getCurrentPosition() > 30 || killMotors) {
        if (CtrlAltDelOpMode.killMotors) {
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
        } else if (gamepad.left_trigger > CtrlAltDelOpMode.TRIGGER_DEADZONE) {
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

        if (aprilTag.tagSeen && !Double.isNaN(aprilTag.z)) {
            double feet = aprilTag.z * 3.28 - 1.5; // There are 3.28ft in a meter, and z is in meters. LAUNCHER_BASE_TICKS is tuned to 2ft, so we subtract 2.
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
