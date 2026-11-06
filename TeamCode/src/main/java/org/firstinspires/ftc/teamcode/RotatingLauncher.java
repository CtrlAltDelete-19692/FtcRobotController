package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.hardware.TouchSensor;

import org.firstinspires.ftc.robotcore.external.Telemetry;

public class RotatingLauncher {
    private final String label;
    public AprilTag aprilTag;
    public Telemetry telemetry;
    public CRServo loader;

    public DcMotorEx launcher;

    private static final double LOADER_IDLE_POWER = 0.1; // Between 0 and 1
    private static final double LOADER_POWER = 0.5; // Between 0 and 1
    private static final int LAUNCH_TIME_MS = 400;
    private static final double LAUNCHER_IDLE_TICKS = 900;  // maintains spin
    public static final double LAUNCHER_BASE_TICKS = 1280;  // full shooting speed from 2ft
    public static final double LAUNCHER_MINIMUM_TICKS = 900;  // full shooting speed from 2ft
    public static final double TICKS_PER_FOOT = 60;  // ticks to add for each extra foot of distance, based on testing
    public static final int LAUNCHER_TICKS_INCREMENTS = 25;  // When manually adjusting launcher, increment / decrement by this amount
    public static final double LAUNCHER_THRESHOLD = 0.94;  // What percentage of launcher set velocity is required to shoot

    public double launcherVelocity = 0;
    public int lvManualAdjustment = 0;
    public int lvGoalDistanceAdjustment = 0;
    public boolean readyToLaunch = false;
    boolean lastLaunch = false;
    boolean runningSequence = false;

    // Turret
    public CRServo turretServo;
    private TouchSensor centerSwitch;
    public static final double TURRET_SERVO_POWER = 1.0;
    private static final double HOME_POWER = 0.25;
    private static final long HOME_CLEAR_TIMEOUT_MS = 500;
    private static final long HOME_FIRST_SEARCH_MS = 750;
    private static final long HOME_REVERSE_SEARCH_MS = 1500;
    private HomeState homeState = HomeState.IDLE;
    private long homeStageStartTime;
    private boolean lastHomeButton;

    private enum HomeState {
        IDLE, CLEARING, SEARCHING_FIRST, SEARCHING_REVERSE,
        HOMED, FAILED, CANCELED, UNAVAILABLE
    }

    long actionStartTime = 0;
    private boolean lastLeftBumper = false;
    private boolean lastRightBumper = false;

    enum ServoState {
        IDLE,
        GOING_OUT,
        RETURNING
    }

    ServoState servoState = ServoState.IDLE;

    public RotatingLauncher(HardwareMap hardwareMap, String label, AprilTag aprilTag,
                            Telemetry telemetry) {
        this.label = label;
        setup(hardwareMap);
        this.aprilTag = aprilTag;
        this.telemetry = telemetry;
    }

    private void setup(HardwareMap hardwareMap) {
        loader = hardwareMap.tryGet(CRServo.class, label + "Loader");
        if (loader != null) {
            loader.setDirection(CRServo.Direction.FORWARD);
            loader.setPower(0);
        }

        launcher = hardwareMap.get(DcMotorEx.class, label + "Flywheel");
        launcher.setDirection(DcMotor.Direction.REVERSE);
        launcher.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        launcher.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        launcher.setVelocity(0);
        PIDFCoefficients newCoeffs = new PIDFCoefficients(14  , 0, 0.8, 13.2);
        launcher.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, newCoeffs);

        turretServo = hardwareMap.tryGet(CRServo.class, label + "RotatingServo");
        if (turretServo != null) {
            turretServo.setDirection(CRServo.Direction.FORWARD);
            turretServo.setPower(0);
            centerSwitch = hardwareMap.tryGet(TouchSensor.class, label + "CenterSwitch");
        }
    }

    public void update(Gamepad gamepad) {
        // Do not move the loader or launcher motors unless we are on the ground!
        //boolean onGround = leftViperSlideMotor.getCurrentPosition() <= 30 && rightViperSlideMotor.getCurrentPosition() <= 30;
        //if (! onGround) {
        //    return;
        //}

        launcher(gamepad);
        loader(gamepad);
        if (gamepad.dpad_up && !lastHomeButton) {
            startHoming();
        }
        lastHomeButton = gamepad.dpad_up;
        turret(gamepad);
    }

    private void loader(Gamepad gamepad) {
        boolean minLaunchSpeed = launcher.getVelocity() >= LAUNCHER_MINIMUM_TICKS;

        boolean aPressed = gamepad.a;
        boolean bPressed = gamepad.b;

        boolean executeLaunch =
                ((aPressed && readyToLaunch) || bPressed)
                        && !gamepad.start
                        && minLaunchSpeed;

        runLoader(executeLaunch);
    }

    private void turret(Gamepad gamepad) {
        if (turretServo == null) return;

        if (CtrlAltDelOpMode.killMotors) {
            if (isHoming()) homeState = HomeState.CANCELED;
            turretServo.setPower(0);
        } else if (gamepad.dpad_left || gamepad.dpad_right) {
            if (isHoming()) homeState = HomeState.CANCELED;
            turretServo.setPower(gamepad.dpad_left ? TURRET_SERVO_POWER : -TURRET_SERVO_POWER);
        } else if (isHoming()) {
            updateHoming();
        } else {
            turretServo.setPower(0);
        }
    }

    public void startHoming() {
        if (turretServo == null || centerSwitch == null) {
            homeState = HomeState.UNAVAILABLE;
            return;
        }
        if (CtrlAltDelOpMode.killMotors) {
            homeState = HomeState.CANCELED;
            turretServo.setPower(0);
            return;
        }

        setHomeState(centerSwitch.isPressed() ? HomeState.CLEARING : HomeState.SEARCHING_FIRST);
    }

    private boolean isHoming() {
        return homeState == HomeState.CLEARING
                || homeState == HomeState.SEARCHING_FIRST
                || homeState == HomeState.SEARCHING_REVERSE;
    }

    private void setHomeState(HomeState state) {
        homeState = state;
        homeStageStartTime = System.currentTimeMillis();
    }

    private void updateHoming() {
        long elapsed = System.currentTimeMillis() - homeStageStartTime;

        if (homeState == HomeState.CLEARING) {
            if (!centerSwitch.isPressed()) {
                setHomeState(HomeState.SEARCHING_FIRST);
                turretServo.setPower(HOME_POWER);
            } else if (elapsed >= HOME_CLEAR_TIMEOUT_MS) {
                homeState = HomeState.FAILED;
                turretServo.setPower(0);
            } else {
                turretServo.setPower(-HOME_POWER);
            }
            return;
        }

        if (centerSwitch.isPressed()) {
            homeState = HomeState.HOMED;
            turretServo.setPower(0);
        } else if (homeState == HomeState.SEARCHING_FIRST) {
            if (elapsed >= HOME_FIRST_SEARCH_MS) {
                setHomeState(HomeState.SEARCHING_REVERSE);
                turretServo.setPower(-HOME_POWER);
            } else {
                turretServo.setPower(HOME_POWER);
            }
        } else if (elapsed >= HOME_REVERSE_SEARCH_MS) {
            homeState = HomeState.FAILED;
            turretServo.setPower(0);
        } else {
            turretServo.setPower(-HOME_POWER);
        }
    }

    public void stopTurret() {
        if (turretServo != null) turretServo.setPower(0);
    }

    // Call this from autonomous when you want to launch
    public void triggerLoader() {
        runLoader(true);
    }

    public void updateLoader() {
        runLoader(false);
    }

    // Shared loader sequence logic
    private void runLoader(boolean executeLaunch) {
        if (CtrlAltDelOpMode.killMotors) {
            if (loader != null) loader.setPower(0);
            servoState = ServoState.IDLE;
            runningSequence = false;
            lastLaunch = executeLaunch;
            return;
        }

        if (loader == null) return;

        int launchTimeMs = LAUNCH_TIME_MS;

        if (executeLaunch && !lastLaunch && !runningSequence) {
            runningSequence = true;
            servoState = ServoState.GOING_OUT;
            actionStartTime = System.currentTimeMillis();
        }

        lastLaunch = executeLaunch;

        long elapsed = System.currentTimeMillis() - actionStartTime;

        switch (servoState) {
            case IDLE:
                loader.setPower(LOADER_IDLE_POWER);
                loader.setDirection(CRServo.Direction.REVERSE);
                break;

            case GOING_OUT:
                loader.setPower(LOADER_POWER);
                loader.setDirection(CRServo.Direction.FORWARD);

                if (elapsed >= launchTimeMs) {
                    servoState = ServoState.RETURNING;
                    actionStartTime = System.currentTimeMillis();
                }
                break;

            case RETURNING:
                loader.setPower(LOADER_POWER);
                loader.setDirection(CRServo.Direction.REVERSE);

                if (elapsed >= launchTimeMs) {
                    servoState = ServoState.IDLE;
                    runningSequence = false;
                }
                break;
        }
    }

    private void launcher(Gamepad gamepad) {
        boolean spinLauncher = gamepad.right_trigger > CtrlAltDelOpMode.TRIGGER_DEADZONE;

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
        if (gamepad.left_bumper && !lastLeftBumper) {
            lvManualAdjustment -= LAUNCHER_TICKS_INCREMENTS;
            if (telemetry != null) {
                telemetry.speak("Slower, " + lvManualAdjustment + " ticks");
            }
        }
        if (gamepad.right_bumper && !lastRightBumper) {
            lvManualAdjustment += LAUNCHER_TICKS_INCREMENTS;
            if (telemetry != null) {
                telemetry.speak("Faster, " + lvManualAdjustment + " ticks");
            }
        }
        lastLeftBumper = gamepad.left_bumper;
        lastRightBumper = gamepad.right_bumper;

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

        if (aprilTag != null && aprilTag.tagSeen && !Double.isNaN(aprilTag.z)) {
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

    public void addTelemetry(Telemetry telemetry) {
        telemetry.addData(
                label + " Flywheel",
                "%.0f / %.0f",
                launcher.getVelocity(),
                launcherVelocity
        );

        telemetry.addData(label + " Ready", readyToLaunch);
        telemetry.addData(label + " Shot Target", "%.0f",
                LAUNCHER_BASE_TICKS + lvManualAdjustment + lvGoalDistanceAdjustment);
        telemetry.addData(label + " Tag Adjustment", lvGoalDistanceAdjustment);
        telemetry.addData(label + " Manual Adjustment", lvManualAdjustment);

        if (loader != null) {
            telemetry.addData(
                    label + " Loader",
                    "%.0f%%",
                    loader.getPower() * 100
            );
        }

        if (turretServo != null) {
            telemetry.addData(
                    label + " Turret",
                    "%.0f%%",
                    turretServo.getPower() * 100
            );
            telemetry.addData(label + " Center Switch",
                    centerSwitch == null ? "Not configured" :
                            centerSwitch.isPressed() ? "Detected" : "Clear");
            telemetry.addData(label + " Home", homeState);
        }
    }
}
