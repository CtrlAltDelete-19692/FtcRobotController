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
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Position;

public class RotatingLauncher {
    private final String label;
    public AprilTag aprilTag;
    public Telemetry telemetry;
    public CRServo loader;
    private Intake feedIntake;

    public DcMotorEx launcher;

    private static final double LOADER_IDLE_POWER = 0; // Between 0 and 1
    private static final double LOADER_POWER = 0.6; // Between 0 and 1d
    private static final double LOADER_CONTINUOUS_POWER = 0.6;
    private static final int LAUNCH_TIME_MS = 400;
    private static final double LAUNCHER_IDLE_TICKS = 600;  // maintains spin
    public static final double LAUNCHER_BASE_TICKS = 1020;  // Base speed; distance adjustment starts beyond 1.5 ft.
    public static final double LAUNCHER_MINIMUM_TICKS = 600;  // full shooting speed from 2ft
    public static final double TICKS_PER_FOOT = 60;  // ticks to add for each extra foot of distance, based on testing
    public static final double NO_TAG_DISTANCE_FEET = 3.5;
    public static final int LAUNCHER_TICKS_INCREMENTS = 25;  // When manually adjusting launcher, increment / decrement by this amount
    public static final double LAUNCHER_THRESHOLD = 0.94;  // What percentage of launcher set velocity is required to shoot
    // Camera-space Y points down and Z points forward. Positive mounting pitch looks upward.
    public static final double CAMERA_HEIGHT_INCHES = 14.0;
    public static final double CAMERA_UPWARD_PITCH_DEGREES = Math.toDegrees(Math.atan2(17.0, 23.0)); // 36.47 degrees; lens normal to 23 mm down / 17 mm deep chamfer.
    public static final double MIN_TARGET_HEIGHT_INCHES = 42.0; // Reject at or below this floor height.
    private static final double HEIGHT_COMPARISON_EPSILON_INCHES = 1e-6; // Absorb rounding at the cutoff.

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
    private static final double LED_BLUE_POSITION = 0.611;
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
    private static final double TRACK_MAX_POWER = 0.4;
    private static final double TRACK_DIRECTION = -1.0; // Reverse if positive tx moves farther from zero.
    private static final double TRACK_DEADBAND_DEGREES = 0.5;
    private static final double TRACK_RESUME_DEGREES = 1.0; // Restart only outside this wider grace zone.
    private static final long TARGET_ACQUISITION_SETTLE_MS = 150;
    // Temporary travel bounds until turret position/limit feedback is available.
    // One left/right/return search per aiming session; never sweep indefinitely.
    private static final double SWEEP_POWER = 0.10;
    private static final long SWEEP_LEG_MS = 1000; // Longer, slower legs approximate the previous travel.
    public static final boolean DEFAULT_SWEEP_ENABLED = true;
    private HomeState homeState = HomeState.IDLE;
    private long homeStageStartTime;
    private double fineApproachDirection;
    private boolean lastHomeButton;
    private boolean sweepEnabled = DEFAULT_SWEEP_ENABLED;
    private boolean sweepAttempted;
    private long sweepStartTime;
    private boolean trackingEnabled;
    private boolean purgeRunning;
    private boolean automaticFeedLatched;
    private TrackingState trackingState = TrackingState.IDLE;
    private int trackedTagId = -1;
    private double trackedTx = Double.NaN;
    private boolean alignmentHeld;
    private long acquisitionSettleUntil;
    private LLResultTypes.FiducialResult speedTarget;
    private boolean usingLimelightResults;

    private enum HomeState {
        IDLE, CLEARING, SEARCHING_FIRST, SEARCHING_REVERSE,
        FINE_CLEARING, FINE_SEARCHING, HOMED, FAILED, CANCELED, UNAVAILABLE
    }

    private enum TrackingState {
        IDLE, TRACKING, ALIGNED, SETTLING, NO_TAG, LOST, SEARCHING, SEARCH_EXHAUSTED, CANCELED
    }

    long actionStartTime = 0;
    private boolean lastLeftBumper = false;
    private boolean lastRightBumper = false;
    private boolean lastSpinTrigger;

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

    public void setSweepEnabled(boolean enabled) {
        sweepEnabled = enabled;
        if (!enabled && trackingState == TrackingState.SEARCHING) {
            // Do not restart a partially completed search if re-enabled while RT is held.
            sweepStartTime = 0;
            trackingState = trackedTagId == -1 ? TrackingState.NO_TAG : TrackingState.LOST;
            if (turretServo != null) turretServo.setPower(0);
        }
    }

    public boolean isSweepEnabled() {
        return sweepEnabled;
    }

    public boolean isLoaderRunning() {
        return loader != null && loader.getPower() > 0;
    }

    public void setFeedIntake(Intake intake) {
        feedIntake = intake;
    }

    public boolean hasLoader() {
        return loader != null;
    }

    public void update(Gamepad gamepad, Gamepad gamepad2, LLResult result, boolean isBlue) {
        // Do not move the loader or launcher motors unless we are on the ground!
        //boolean onGround = leftViperSlideMotor.getCurrentPosition() <= 30 && rightViperSlideMotor.getCurrentPosition() <= 30;
        //if (! onGround) {
        //    return;
        //}

        if (gamepad.dpad_up && !lastHomeButton) {
            startHoming();
        }
        lastHomeButton = gamepad.dpad_up;
        turret(gamepad, gamepad2, result, isBlue);
        // Resolve aiming first so speed uses the same tag, including on acquisition/loss.
        usingLimelightResults = true;
        speedTarget = selectTarget(result, isBlue, trackingEnabled ? trackedTagId : -1);
        if (purgeRunning) {
            applyPurge();
            updateLed();
            return;
        }
        launcher(gamepad, gamepad2);
        loader(gamepad, gamepad2, result, isBlue);
        updateLed();
    }

    private void loader(Gamepad gamepad, Gamepad gamepad2, LLResult result, boolean isBlue) {
        boolean manualFeed = /* gamepad.b || */ gamepad2 != null && gamepad2.b;
        boolean automaticFeed = /* gamepad.a || */ gamepad2 != null && gamepad2.a;
        boolean readyToStart = readyToLaunch
                && launcher.getVelocity() >= LAUNCHER_MINIMUM_TICKS
                && hasAllianceTag(result, isBlue);
        // Once A starts feeding, keep it running through brief speed or tag losses.
        if (!automaticFeed || !isShootTriggerPressed(gamepad, gamepad2) || !trackingEnabled) {
            automaticFeedLatched = false;
        } else if (readyToStart) {
            automaticFeedLatched = true;
        }
        boolean feed = manualFeed || automaticFeedLatched;
        servoState = ServoState.IDLE;
        runningSequence = false;
        lastLaunch = false;
        setLoaderRunning(feed);
    }

    private boolean hasAllianceTag(LLResult result, boolean isBlue) {
        return selectTarget(result, isBlue, -1) != null;
    }

    private boolean isShootTriggerPressed(Gamepad gamepad, Gamepad gamepad2) {
        return /* gamepad.right_trigger > CtrlAltDelOpMode.TRIGGER_DEADZONE || */
                gamepad2 != null && gamepad2.right_trigger > CtrlAltDelOpMode.TRIGGER_DEADZONE;
    }

    private void turret(Gamepad gamepad, Gamepad gamepad2, LLResult result, boolean isBlue) {
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
        } else {
            boolean shootTrigger = isShootTriggerPressed(gamepad, gamepad2);
            if (shootTrigger && !lastSpinTrigger
                    && selectTarget(result, isBlue, trackedTagId) == null) startAiming();
            trackAvailableTag(result, isBlue, shootTrigger);
        }
    }

    private void trackAvailableTag(LLResult result, boolean isBlue, boolean allowSearch) {
        LLResultTypes.FiducialResult visible = selectTarget(result, isBlue, -1);
        if (!trackingEnabled) {
            if (visible == null && !allowSearch) {
                turretServo.setPower(0);
                return;
            }
            startAiming();
        } else if (visible != null
                && selectTarget(result, isBlue, trackedTagId) == null) {
            // Follow another eligible visible tag when the previous one is no longer available.
            startAiming();
        }
        updateTracking(result, isBlue, allowSearch);
    }

    public void updateVisibleTagTracking(LLResult result, boolean isBlue) {
        if (CtrlAltDelOpMode.killMotors || turretServo == null) {
            stopTurret();
        } else if (!isHoming()) {
            trackAvailableTag(result, isBlue, false);
        }
        usingLimelightResults = true;
        speedTarget = selectTarget(result, isBlue, trackingEnabled ? trackedTagId : -1);
        updateLed();
    }

    // With a locked ID, never substitute another tag when that target disappears.
    // Otherwise prefer the inner tags, breaking ties by camera horizontal center.
    private LLResultTypes.FiducialResult selectTarget(LLResult result, boolean isBlue, int targetId) {
        if (result == null || !result.isValid() || result.getFiducialResults() == null) return null;
        LLResultTypes.FiducialResult selected = null;
        double closestTx = Double.POSITIVE_INFINITY;
        boolean selectedPreferred = false;
        for (LLResultTypes.FiducialResult tag : result.getFiducialResults()) {
            int id = tag.getFiducialId();
            double tx = tag.getTargetXDegrees();
            if ((isBlue ? id < 38 || id > 45 : id < 30 || id > 37)
                    || !Double.isFinite(tx) || !isTagAboveMinimumHeight(tag)) continue;
            if (targetId != -1) {
                if (id == targetId) return tag;
            } else {
                boolean preferred = isPreferredTag(id, isBlue);
                if (selected == null || (preferred && !selectedPreferred)
                        || (preferred == selectedPreferred && Math.abs(tx) < closestTx)) {
                    closestTx = Math.abs(tx);
                    selected = tag;
                    selectedPreferred = preferred;
                }
            }
        }
        return selected;
    }

    private boolean isPreferredTag(int id, boolean isBlue) {
        return isBlue ? id == 39 || id == 40 || id == 43 || id == 44
                : id == 31 || id == 32 || id == 35 || id == 36;
    }

    public double getTagHeightInches(LLResultTypes.FiducialResult tag) {
        if (tag.getTargetPoseCameraSpace() == null
                || tag.getTargetPoseCameraSpace().getPosition() == null) return Double.NaN;
        Position position = tag.getTargetPoseCameraSpace().getPosition().toUnit(DistanceUnit.INCH);
        if (!Double.isFinite(position.y) || !Double.isFinite(position.z) || position.z <= 0) return Double.NaN;
        double pitch = Math.toRadians(CAMERA_UPWARD_PITCH_DEGREES);
        return CAMERA_HEIGHT_INCHES - position.y * Math.cos(pitch) + position.z * Math.sin(pitch);
    }

    public boolean isTagAboveMinimumHeight(LLResultTypes.FiducialResult tag) {
        double height = getTagHeightInches(tag);
        return Double.isFinite(height) && height > MIN_TARGET_HEIGHT_INCHES + HEIGHT_COMPARISON_EPSILON_INCHES;
    }

    private void updateTracking(LLResult result, boolean isBlue) {
        updateTracking(result, isBlue, true);
    }

    private void updateTracking(LLResult result, boolean isBlue, boolean allowSearch) {
        if (trackedTagId != -1 && result != null && result.isValid() && result.getFiducialResults() != null) {
            for (LLResultTypes.FiducialResult tag : result.getFiducialResults()) {
                if (tag.getFiducialId() != trackedTagId) continue;
                double height = getTagHeightInches(tag);
                if (Double.isFinite(height) && height <= MIN_TARGET_HEIGHT_INCHES + HEIGHT_COMPARISON_EPSILON_INCHES) {
                    trackedTagId = -1;
                    trackedTx = Double.NaN;
                }
                break;
            }
        }
        LLResultTypes.FiducialResult target = selectTarget(result, isBlue, trackedTagId);
        if (target == null) {
            trackingState = trackedTagId == -1 ? TrackingState.NO_TAG : TrackingState.LOST;
            if (allowSearch) updateSearchSweep();
            else turretServo.setPower(0);
            return;
        }

        if (trackedTagId == -1) {
            trackedTagId = target.getFiducialId();
            acquisitionSettleUntil = System.currentTimeMillis() + TARGET_ACQUISITION_SETTLE_MS;
            alignmentHeld = false;
        }
        // A detected tag ends this session's search budget, even if it is lost later.
        sweepAttempted = true;
        sweepStartTime = 0;

        trackedTx = target.getTargetXDegrees();
        if (System.currentTimeMillis() < acquisitionSettleUntil) {
            trackingState = TrackingState.SETTLING;
            turretServo.setPower(0);
            return;
        }
        double error = Math.abs(trackedTx);
        if (error <= TRACK_DEADBAND_DEGREES) alignmentHeld = true;
        else if (error > TRACK_RESUME_DEGREES) alignmentHeld = false;
        if (alignmentHeld) {
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
    }

    private void updateSearchSweep() {
        if (!sweepEnabled) {
            turretServo.setPower(0);
            return;
        }
        if (!sweepAttempted) {
            sweepAttempted = true;
            sweepStartTime = System.currentTimeMillis();
        }
        long elapsed = System.currentTimeMillis() - sweepStartTime;
        if (sweepStartTime == 0 || elapsed >= SWEEP_LEG_MS * 4) {
            trackingState = TrackingState.SEARCH_EXHAUSTED;
            turretServo.setPower(0);
            return;
        }
        trackingState = TrackingState.SEARCHING;
        double direction = elapsed < SWEEP_LEG_MS || elapsed >= SWEEP_LEG_MS * 3 ? 1 : -1;
        turretServo.setPower(direction * SWEEP_POWER);
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
        return trackingState == TrackingState.TRACKING || trackingState == TrackingState.ALIGNED
                || trackingState == TrackingState.SETTLING;
    }

    private boolean isLedReady() {
        return speedTarget != null && !CtrlAltDelOpMode.killMotors
                && launcherVelocity > LAUNCHER_IDLE_TICKS && upToSpeed();
    }

    private double getLedPosition() {
        if (isLedReady()) return LED_GREEN_POSITION;
        boolean ramping = !CtrlAltDelOpMode.killMotors && launcherVelocity > LAUNCHER_IDLE_TICKS
                && !upToSpeed();
        double color = ramping ? LED_BLUE_POSITION : LED_RED_POSITION;
        boolean blinkOff = speedTarget != null
                && System.currentTimeMillis() % LED_BLINK_PERIOD_MS >= LED_BLINK_PERIOD_MS / 2;
        return blinkOff ? LED_OFF_POSITION : color;
    }

    private void updateLed() {
        if (led != null) led.setPosition(getLedPosition());
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
        trackingState = TrackingState.IDLE;
        trackedTagId = -1;
        trackedTx = Double.NaN;
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
        boolean spinLauncher = isShootTriggerPressed(gamepad, gamepad2);
        lastSpinTrigger = spinLauncher;

        //if (leftViperSlideMotor.getCurrentPosition() > 30 || rightViperSlideMotor.getCurrentPosition() > 30 || killMotors) {
        if (CtrlAltDelOpMode.killMotors) {
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
        int distanceAdjustment = Math.max(0, lvGoalDistanceAdjustment);

        // Set launch velocity
        if (spinLauncher) {
            launcherVelocity = LAUNCHER_BASE_TICKS + lvManualAdjustment + distanceAdjustment;
        }

        launcher.setVelocity(launcherVelocity);
        readyToLaunch = spinLauncher && trackingEnabled && trackingState == TrackingState.ALIGNED
                && lvGoalDistanceAdjustment >= 0 && upToSpeed();
    }

    public void stopLauncher() {
        launcher.setVelocity(0);
        launcherVelocity = 0;
        readyToLaunch = false;
        updateLed();
    }

    public void setAutonomousFlywheelVelocity(double ticksPerSecond) {
        launcherVelocity = CtrlAltDelOpMode.killMotors ? 0 : Math.max(0, ticksPerSecond);
        launcher.setVelocity(launcherVelocity);
    }

    public void startAutonomousAiming() {
        startAiming();
    }

    private void startAiming() {
        trackedTagId = -1;
        trackedTx = Double.NaN;
        alignmentHeld = false;
        acquisitionSettleUntil = 0;
        trackingEnabled = turretServo != null && !CtrlAltDelOpMode.killMotors;
        trackingState = trackingEnabled ? TrackingState.NO_TAG : TrackingState.IDLE;
        sweepAttempted = false;
        sweepStartTime = 0;
        readyToLaunch = false;
    }

    public void updateAutonomousShooting(LLResult result, boolean isBlue) {
        updateAutonomousShooting(result, isBlue, Double.NaN);
    }

    public void updateAutonomousShooting(LLResult result, boolean isBlue, double fixedVelocity) {
        if (CtrlAltDelOpMode.killMotors) {
            stopTurret();
            stopLauncher();
            readyToLaunch = false;
            return;
        }
        if (trackingEnabled) updateTracking(result, isBlue, false);
        usingLimelightResults = true;
        speedTarget = selectTarget(result, isBlue, trackingEnabled ? trackedTagId : -1);
        lvGoalDistanceAdjustment = getGoalDistanceAdjustment();
        setAutonomousFlywheelVelocity(Double.isFinite(fixedVelocity) ? fixedVelocity
                : LAUNCHER_BASE_TICKS + Math.max(0, lvGoalDistanceAdjustment));
        readyToLaunch = (speedTarget == null || trackingEnabled && trackingState == TrackingState.ALIGNED
                && lvGoalDistanceAdjustment >= 0) && upToSpeed();
        updateLed();
    }

    public boolean isUsingNoTagFallback() {
        return usingLimelightResults && speedTarget == null;
    }

    public boolean isAimingTargetVisible() {
        return trackingEnabled && isTrackedTagVisible();
    }

    public void setLoaderRunning(boolean running) {
        boolean requested = running && loader != null && !CtrlAltDelOpMode.killMotors;
        if (feedIntake != null) feedIntake.onLoaderRunning(requested);
        if (loader == null) return;
        loader.setDirection(CRServo.Direction.FORWARD);
        loader.setPower(requested ? LOADER_CONTINUOUS_POWER : 0);
    }

    public void setPurgeRunning(boolean running) {
        if (purgeRunning == running) return;
        purgeRunning = running;
        if (running) {
            automaticFeedLatched = false;
            setLoaderRunning(false);
            applyPurge();
        } else {
            if (loader != null) loader.setPower(0);
            stopLauncher();
        }
    }

    private void applyPurge() {
        if (CtrlAltDelOpMode.killMotors) {
            readyToLaunch = false;
            launcherVelocity = 0;
            launcher.setVelocity(0);
            if (loader != null) loader.setPower(0);
            return;
        }
        readyToLaunch = false;
        launcherVelocity = -LAUNCHER_IDLE_TICKS;
        launcher.setVelocity(launcherVelocity);
        if (loader != null) {
            loader.setDirection(CRServo.Direction.FORWARD);
            loader.setPower(-LOADER_CONTINUOUS_POWER);
        }
    }

    public void setAutonomousLoaderRunning(boolean running) {
        setLoaderRunning(running);
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
        double zMeters = Double.NaN;
        if (usingLimelightResults) {
            if (speedTarget != null && speedTarget.getTargetPoseCameraSpace() != null
                    && speedTarget.getTargetPoseCameraSpace().getPosition() != null) {
                zMeters = speedTarget.getTargetPoseCameraSpace().getPosition().z;
            }
        } else if (aprilTag != null && aprilTag.tagSeen) {
            // Preserve the wrapper-based path for callers that do not use update(..., result, ...).
            zMeters = aprilTag.z;
        }
        boolean noTag = usingLimelightResults ? speedTarget == null : aprilTag == null || !aprilTag.tagSeen;
        if (!noTag && (!Double.isFinite(zMeters) || zMeters <= 0)) return -1;
        // Preserve the existing calibration: add 60 ticks/s per foot beyond 1.5 ft.
        double extraFeet = Math.max(0, (noTag ? NO_TAG_DISTANCE_FEET : zMeters * 3.28) - 1.5);
        int adjustment = (int) (extraFeet * TICKS_PER_FOOT);
        return adjustment - adjustment % 10; // Reduce speed jitter from small distance changes.
    }

    public boolean upToSpeed() {
        return launcher.getVelocity() >= (launcherVelocity * LAUNCHER_THRESHOLD);
    }

    public void addCompetitionTelemetry(Telemetry telemetry) {
        telemetry.addData("AprilTag", speedTarget == null ? "" : "✅ " + speedTarget.getFiducialId());
        String speedState = CtrlAltDelOpMode.killMotors || launcherVelocity == 0 ? "❌ Stopped" :
                launcherVelocity < 0 ? "↩ Reverse" :
                        launcherVelocity <= LAUNCHER_IDLE_TICKS ? "Idle" :
                                upToSpeed() ? "✅ At speed" : "🔵 Spinning up";
        telemetry.addData("Flywheel", "%.0f / %.0f ticks/s  %s",
                launcher.getVelocity(), launcherVelocity, speedState);

        if (isHoming()) {
            telemetry.addLine("⌛ Turret: Homing");
        } else if (trackingEnabled && isTrackedTagVisible()) {
            telemetry.addData("🎯 Turret", "Tag %d  tx %+.1f°", trackedTagId, trackedTx);
        } else if (trackingEnabled) {
            telemetry.addLine("🔍 Turret: " + (trackingState == TrackingState.LOST ? "Tag lost" : "Waiting for tag"));
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
        if (speedTarget != null) {
            telemetry.addData("Speed target tag", speedTarget.getFiducialId());
            telemetry.addData("Speed target height", "%.1f in (must be > %.0f)",
                    getTagHeightInches(speedTarget), MIN_TARGET_HEIGHT_INCHES);
        }
        if (speedTarget == null) {
            telemetry.addData("Distance input", "No eligible tag; using %.1f ft fallback", NO_TAG_DISTANCE_FEET);
        } else if (speedTarget.getTargetPoseCameraSpace() == null
                || speedTarget.getTargetPoseCameraSpace().getPosition() == null) {
            telemetry.addData("Distance input", "Camera-space pose unavailable");
        } else {
            double zMeters = speedTarget.getTargetPoseCameraSpace().getPosition().z;
            telemetry.addData("Speed target distance", "%.3f m / %.2f ft", zMeters, zMeters * 3.28);
            telemetry.addData("Distance input", Double.isFinite(zMeters) && zMeters > 0 ? "Valid" : "Invalid distance");
        }
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
        telemetry.addData("LED", led == null ? "Not configured" : isLedReady() ? "Green" :
                (speedTarget != null ? "Flashing " : "")
                        + (!CtrlAltDelOpMode.killMotors && launcherVelocity > LAUNCHER_IDLE_TICKS
                        && !upToSpeed() ? "blue" : "red"));
    }
}
