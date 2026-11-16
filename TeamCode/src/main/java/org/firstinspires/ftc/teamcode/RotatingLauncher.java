package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.TouchSensor;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.Telemetry;

public class RotatingLauncher {
    private final String label;
    public AprilTag aprilTag;
    public Telemetry telemetry;
    public CRServo loader;

    public DcMotorEx launcher;

    private static final double LOADER_IDLE_POWER = 0; // Between 0 and 1
    private static final double LOADER_POWER = 0.4; // Between 0 and 1
    private static final double LOADER_CONTINUOUS_POWER = 0.6;
    private static final int LAUNCH_TIME_MS = 400;
    private static final double LAUNCHER_IDLE_TICKS = 600;  // maintains spin
    public static final double LAUNCHER_BASE_TICKS = 1280;  // full shooting speed from 2ft
    public static final double LAUNCHER_MINIMUM_TICKS = 600;  // full shooting speed from 2ft
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
    private Servo led;
    public static final double TURRET_SERVO_POWER = 1.0;
    private static final double LED_RED_POSITION = 0.28;
    private static final double LED_GREEN_POSITION = 0.50;
    private static final double LED_OFF_POSITION = 0.0;
    private static final long LED_BLINK_PERIOD_MS = 1000;
    private static final double HOME_POWER = 0.35;
    private static final double HOME_FINE_POWER = 0.10;
    private static final long HOME_CLEAR_TIMEOUT_MS = 500;
    private static final long HOME_FIRST_SEARCH_MS = 850;
    private static final long HOME_REVERSE_SEARCH_MS = 1300;
    private static final long HOME_FINE_CLEAR_TIMEOUT_MS = 1500;
    private static final long HOME_FINE_SEARCH_TIMEOUT_MS = 2000;
    private static final double TRACK_KP = 0.025;
    private static final double TRACK_MIN_POWER = 0.1;
    private static final double TRACK_MAX_POWER = 0.5;
    private static final double TRACK_DIRECTION = -1.0; // Reverse if positive tx moves farther from zero.
    private static final double TRACK_DEADBAND_DEGREES = 1.0;
    private static final long TRACK_TIMEOUT_MS = 25000;
    private HomeState homeState = HomeState.IDLE;
    private long homeStageStartTime;
    private double fineApproachDirection;
    private boolean lastHomeButton;
    private boolean lastTrackButton;
    private boolean trackingEnabled;
    private TrackingState trackingState = TrackingState.IDLE;
    private int trackedTagId = -1;
    private double trackedTx = Double.NaN;
    private long trackingStartTime;

    private enum HomeState {
        IDLE, CLEARING, SEARCHING_FIRST, SEARCHING_REVERSE,
        FINE_CLEARING, FINE_SEARCHING, HOMED, FAILED, CANCELED, UNAVAILABLE
    }

    private enum TrackingState {
        IDLE, TRACKING, ALIGNED, NO_TAG, LOST, TIMED_OUT, CANCELED
    }

    long actionStartTime = 0;
    private boolean lastLeftBumper = false;
    private boolean lastRightBumper = false;
    private boolean lastFlywheelButton;
    private boolean lastSpinTrigger;
    private boolean flywheelEnabled = true;

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
        PIDFCoefficients newCoeffs = new PIDFCoefficients(14  , 0, 0.8, 18);
        launcher.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, newCoeffs);

        turretServo = hardwareMap.tryGet(CRServo.class, label + "RotatingServo");
        if (turretServo != null) {
            turretServo.setDirection(CRServo.Direction.FORWARD);
            turretServo.setPower(0);
            centerSwitch = hardwareMap.tryGet(TouchSensor.class, label + "CenterSwitch");
        }

        led = hardwareMap.tryGet(Servo.class, label + "Led");
        updateLed();
    }

    public void update(Gamepad gamepad) {
        update(gamepad, null, null, false);
    }

    public void update(Gamepad gamepad, LLResult result, boolean isBlue) {
        update(gamepad, null, result, isBlue);
    }

    public void updateEnabled(Gamepad gamepad) {
        if (gamepad.right_bumper && !lastFlywheelButton) flywheelEnabled = !flywheelEnabled;
        lastFlywheelButton = gamepad.right_bumper;
    }

    public boolean isLoaderRunning() {
        return loader != null && loader.getPower() > 0;
    }

    public boolean hasLoader() {
        return loader != null;
    }

    public boolean isFlywheelEnabled() {
        return flywheelEnabled;
    }

    public void update(Gamepad gamepad, Gamepad gamepad2, LLResult result, boolean isBlue) {
        // Do not move the loader or launcher motors unless we are on the ground!
        //boolean onGround = leftViperSlideMotor.getCurrentPosition() <= 30 && rightViperSlideMotor.getCurrentPosition() <= 30;
        //if (! onGround) {
        //    return;
        //}

        updateEnabled(gamepad);
        launcher(gamepad, gamepad2);
        loader(gamepad, gamepad2, result, isBlue);
        if (gamepad.dpad_up && !lastHomeButton) {
            startHoming();
        }
        lastHomeButton = gamepad.dpad_up;
        turret(gamepad, result, isBlue);
        updateLed();
    }

    private void loader(Gamepad gamepad, Gamepad gamepad2, LLResult result, boolean isBlue) {
        boolean manualFeed = gamepad.b || gamepad2 != null && gamepad2.b;
        boolean automaticFeed = gamepad.a || gamepad2 != null && gamepad2.a;
        boolean feed = manualFeed || automaticFeed && readyToLaunch
                && launcher.getVelocity() >= LAUNCHER_MINIMUM_TICKS
                && hasAllianceTag(result, isBlue);
        servoState = ServoState.IDLE;
        runningSequence = false;
        lastLaunch = false;
        setLoaderRunning(feed && !gamepad.start && (gamepad2 == null || !gamepad2.start));
    }

    private boolean hasAllianceTag(LLResult result, boolean isBlue) {
        if (result == null || !result.isValid() || result.getFiducialResults() == null) return false;
        for (LLResultTypes.FiducialResult tag : result.getFiducialResults()) {
            int id = tag.getFiducialId();
            if (isBlue ? id >= 38 && id <= 45 : id >= 30 && id <= 37) return true;
        }
        return false;
    }

    private void turret(Gamepad gamepad, LLResult result, boolean isBlue) {
        boolean trackPressed = gamepad.dpad_down;
        if (trackPressed && !lastTrackButton && !isHoming()) {
            trackingEnabled = !trackingEnabled;
            if (trackingEnabled) {
                trackingState = TrackingState.NO_TAG;
                trackedTagId = -1;
                trackedTx = Double.NaN;
            } else {
                trackingState = TrackingState.IDLE;
                trackedTagId = -1;
                trackedTx = Double.NaN;
            }
        }
        lastTrackButton = trackPressed;
        if (turretServo == null) {
            trackingEnabled = false;
            return;
        }

        if (CtrlAltDelOpMode.killMotors) {
            if (isHoming()) homeState = HomeState.CANCELED;
            cancelTracking();
            turretServo.setPower(0);
        } else if (gamepad.dpad_left || gamepad.dpad_right) {
            if (isHoming()) homeState = HomeState.CANCELED;
            cancelTracking();
            turretServo.setPower(gamepad.dpad_left ? TURRET_SERVO_POWER : -TURRET_SERVO_POWER);
        } else if (isHoming()) {
            cancelTracking();
            updateHoming();
        } else if (trackingEnabled) {
            updateTracking(result, isBlue);
        } else {
            turretServo.setPower(0);
        }
    }

    private void updateTracking(LLResult result, boolean isBlue) {
        if (trackedTagId != -1 && System.currentTimeMillis() - trackingStartTime >= TRACK_TIMEOUT_MS) {
            trackingEnabled = false;
            trackingState = TrackingState.TIMED_OUT;
            turretServo.setPower(0);
            return;
        }
        if (result == null || !result.isValid() || result.getFiducialResults() == null) {
            trackingState = trackedTagId == -1 ? TrackingState.NO_TAG : TrackingState.LOST;
            turretServo.setPower(0);
            return;
        }

        if (trackedTagId == -1) {
            double closestTx = Double.POSITIVE_INFINITY;
            for (LLResultTypes.FiducialResult tag : result.getFiducialResults()) {
                int id = tag.getFiducialId();
                double tx = tag.getTargetXDegrees();
                if ((isBlue ? id < 38 || id > 45 : id < 30 || id > 37)
                        || !Double.isFinite(tx)) continue;
                if (Math.abs(tx) < closestTx) {
                    closestTx = Math.abs(tx);
                    trackedTagId = id;
                }
            }
            if (trackedTagId != -1) trackingStartTime = System.currentTimeMillis();
        }

        for (LLResultTypes.FiducialResult tag : result.getFiducialResults()) {
            if (tag.getFiducialId() != trackedTagId) continue;
            trackedTx = tag.getTargetXDegrees();
            if (!Double.isFinite(trackedTx)) break;
            if (Math.abs(trackedTx) <= TRACK_DEADBAND_DEGREES) {
                trackingState = TrackingState.ALIGNED;
                turretServo.setPower(0);
            } else {
                trackingState = TrackingState.TRACKING;
                double power = Range.clip(TRACK_KP * trackedTx, -TRACK_MAX_POWER, TRACK_MAX_POWER);
                if (Math.abs(power) < TRACK_MIN_POWER) {
                    power = Math.copySign(TRACK_MIN_POWER, power);
                }
                turretServo.setPower(TRACK_DIRECTION * power);
            }
            return;
        }

        trackingState = trackedTagId == -1 ? TrackingState.NO_TAG : TrackingState.LOST;
        turretServo.setPower(0);
    }

    private void cancelTracking() {
        if (trackingEnabled) {
            trackingEnabled = false;
            trackingState = TrackingState.CANCELED;
        }
    }

    public boolean isTrackingEnabled() {
        return trackingEnabled;
    }

    private boolean isTrackedTagVisible() {
        return trackingState == TrackingState.TRACKING || trackingState == TrackingState.ALIGNED;
    }

    private void updateLed() {
        if (led != null) {
            led.setPosition(!trackingEnabled ? LED_RED_POSITION :
                    isTrackedTagVisible() || System.currentTimeMillis() % LED_BLINK_PERIOD_MS < LED_BLINK_PERIOD_MS / 2
                            ? LED_GREEN_POSITION : LED_OFF_POSITION);
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
                || homeState == HomeState.SEARCHING_REVERSE
                || homeState == HomeState.FINE_CLEARING
                || homeState == HomeState.FINE_SEARCHING;
    }

    private void setHomeState(HomeState state) {
        homeState = state;
        homeStageStartTime = System.currentTimeMillis();
    }

    private void startFineHoming(double searchDirection) {
        fineApproachDirection = -searchDirection;
        setHomeState(HomeState.FINE_CLEARING);
        turretServo.setPower(searchDirection * HOME_FINE_POWER);
    }

    private void updateHoming() {
        long elapsed = System.currentTimeMillis() - homeStageStartTime;

        switch (homeState) {
            case CLEARING:
                if (!centerSwitch.isPressed()) {
                    setHomeState(HomeState.SEARCHING_FIRST);
                    turretServo.setPower(HOME_POWER);
                } else if (elapsed >= HOME_CLEAR_TIMEOUT_MS) {
                    homeState = HomeState.FAILED;
                    turretServo.setPower(0);
                } else {
                    turretServo.setPower(-HOME_POWER);
                }
                break;

            case SEARCHING_FIRST:
                if (centerSwitch.isPressed()) {
                    startFineHoming(1.0);
                } else if (elapsed >= HOME_FIRST_SEARCH_MS) {
                    setHomeState(HomeState.SEARCHING_REVERSE);
                    turretServo.setPower(-HOME_POWER);
                } else {
                    turretServo.setPower(HOME_POWER);
                }
                break;

            case SEARCHING_REVERSE:
                if (centerSwitch.isPressed()) {
                    startFineHoming(-1.0);
                } else if (elapsed >= HOME_REVERSE_SEARCH_MS) {
                    homeState = HomeState.FAILED;
                    turretServo.setPower(0);
                } else {
                    turretServo.setPower(-HOME_POWER);
                }
                break;

            case FINE_CLEARING:
                if (!centerSwitch.isPressed()) {
                    setHomeState(HomeState.FINE_SEARCHING);
                    turretServo.setPower(fineApproachDirection * HOME_FINE_POWER);
                } else if (elapsed >= HOME_FINE_CLEAR_TIMEOUT_MS) {
                    homeState = HomeState.FAILED;
                    turretServo.setPower(0);
                } else {
                    turretServo.setPower(-fineApproachDirection * HOME_FINE_POWER);
                }
                break;

            case FINE_SEARCHING:
                if (centerSwitch.isPressed()) {
                    homeState = HomeState.HOMED;
                    turretServo.setPower(0);
                } else if (elapsed >= HOME_FINE_SEARCH_TIMEOUT_MS) {
                    homeState = HomeState.FAILED;
                    turretServo.setPower(0);
                } else {
                    turretServo.setPower(fineApproachDirection * HOME_FINE_POWER);
                }
                break;

            default:
                turretServo.setPower(0);
        }
    }

    public void stopTurret() {
        trackingEnabled = false;
        if (turretServo != null) turretServo.setPower(0);
        updateLed();
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

    private void launcher(Gamepad gamepad, Gamepad gamepad2) {
        boolean spinLauncher = gamepad.right_trigger > CtrlAltDelOpMode.TRIGGER_DEADZONE
                || gamepad2 != null && gamepad2.right_trigger > CtrlAltDelOpMode.TRIGGER_DEADZONE;
        if (spinLauncher && !lastSpinTrigger) flywheelEnabled = true;
        lastSpinTrigger = spinLauncher;

        //if (leftViperSlideMotor.getCurrentPosition() > 30 || rightViperSlideMotor.getCurrentPosition() > 30 || killMotors) {
        if (CtrlAltDelOpMode.killMotors || !flywheelEnabled) {
            readyToLaunch = false;
            launcherVelocity = 0;
            launcher.setVelocity(0);
            return;
        }

        launcherVelocity = LAUNCHER_IDLE_TICKS;

        // Manual launch velocity adjustment
        boolean leftBumper = gamepad2 != null && gamepad2.left_bumper;
        boolean rightBumper = gamepad2 != null && gamepad2.right_bumper;
        if (leftBumper && !lastLeftBumper) {
            lvManualAdjustment -= LAUNCHER_TICKS_INCREMENTS;
            if (telemetry != null) {
                telemetry.speak("Slower, " + lvManualAdjustment + " ticks");
            }
        }
        if (rightBumper && !lastRightBumper) {
            lvManualAdjustment += LAUNCHER_TICKS_INCREMENTS;
            if (telemetry != null) {
                telemetry.speak("Faster, " + lvManualAdjustment + " ticks");
            }
        }
        lastLeftBumper = leftBumper;
        lastRightBumper = rightBumper;

        lvGoalDistanceAdjustment = getGoalDistanceAdjustment();

        // Set launch velocity
        if (spinLauncher) {
            launcherVelocity = LAUNCHER_BASE_TICKS + lvManualAdjustment + lvGoalDistanceAdjustment;
        } else if (gamepad.left_trigger > CtrlAltDelOpMode.TRIGGER_DEADZONE) {
            launcherVelocity = (LAUNCHER_BASE_TICKS + lvManualAdjustment + lvGoalDistanceAdjustment) * -1;
        }

        launcher.setVelocity(launcherVelocity);
        readyToLaunch = spinLauncher && upToSpeed();
    }

    public void stopLauncher() {
        launcher.setVelocity(0);
        launcherVelocity = 0;
    }

    public void setAutonomousFlywheelVelocity(double ticksPerSecond) {
        launcherVelocity = CtrlAltDelOpMode.killMotors ? 0 : Math.max(0, ticksPerSecond);
        launcher.setVelocity(launcherVelocity);
    }

    public void setLoaderRunning(boolean running) {
        if (loader == null) return;
        loader.setDirection(CRServo.Direction.FORWARD);
        loader.setPower(running && !CtrlAltDelOpMode.killMotors ? LOADER_CONTINUOUS_POWER : 0);
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

    public void addCompetitionTelemetry(Telemetry telemetry) {
        if (flywheelEnabled && !CtrlAltDelOpMode.killMotors) {
            String speedState = readyToLaunch ? "✅ Ready" :
                    launcherVelocity < 0 ? "↩ Reverse" :
                            launcherVelocity > LAUNCHER_IDLE_TICKS ? "⌛ Spinning up" : "Idle";
            telemetry.addData("Flywheel", "%.0f / %.0f ticks/s  %s",
                    launcher.getVelocity(), launcherVelocity, speedState);
        }

        if (isHoming()) {
            telemetry.addLine("⌛ Turret: Homing");
        } else if (trackingEnabled && isTrackedTagVisible()) {
            telemetry.addData("🎯 Turret", "Tag %d  tx %+.1f°", trackedTagId, trackedTx);
        } else if (trackingEnabled) {
            telemetry.addLine("🔍 Turret: " + (trackingState == TrackingState.LOST ? "Tag lost" : "Searching"));
        } else if (homeState == HomeState.FAILED) {
            telemetry.addLine("⚠ Turret: Home failed");
        } else if (homeState == HomeState.UNAVAILABLE) {
            telemetry.addLine("⚠ Turret: Home unavailable");
        } else {
            telemetry.addLine("🎯 Turret: Tracking off");
        }
    }

    public void addTelemetry(Telemetry telemetry) {
        telemetry.addData("Manual speed adjustment", "%+d ticks/s", lvManualAdjustment);
        telemetry.addData("Tag speed adjustment", lvGoalDistanceAdjustment == -1 ? "Unavailable" :
                lvGoalDistanceAdjustment + " ticks/s");
        telemetry.addData("Loader power", loader == null ? "Not configured" :
                String.format("%.0f%%", loader.getPower() * 100));
        if (turretServo != null) {
            telemetry.addData("Turret power", "%.0f%%", turretServo.getPower() * 100);
            telemetry.addData("Center switch", centerSwitch == null ? "Not configured" :
                    centerSwitch.isPressed() ? "Detected" : "Clear");
            telemetry.addData("Home", homeState);
        }
        telemetry.addData("Tracking detail", trackingState);
        if (trackedTagId != -1 && !isTrackedTagVisible()) {
            telemetry.addData("Tracked tag ID", trackedTagId);
            telemetry.addData("Last tracking tx", "%.1f deg", trackedTx);
        }
        telemetry.addData("LED", led == null ? "Not configured" :
                !trackingEnabled ? "Red" : isTrackedTagVisible() ? "Green" : "Blinking green");
    }
}
