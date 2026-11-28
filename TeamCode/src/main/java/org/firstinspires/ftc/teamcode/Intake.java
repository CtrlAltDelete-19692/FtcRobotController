package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;

public class Intake {

    public DcMotor pickupMotor;
    private final DcMotorEx intakeEncoder;
    private final DcMotor hopper;
    private final DcMotorEx hopperEncoder;

    public static final double INTAKE_POWER = 0.6;  // Default power for pickup intake, 0 to 1
    public static final double HOPPER_POWER = 0.5; // Default power for hopper, 0 to 1
    private static final double STALL_SPEED_TICKS_PER_SECOND = 5;
    public static final double HOPPER_SLOWDOWN_FRACTION = 0.35; // Stop below 35% of this run's peak speed.
    private static final long HOPPER_SLOWDOWN_CONFIRM_MS = 250;
    public static final long HOPPER_STALL_RESTART_MS = 3000; // Wait after stopping before retrying.
    private static final long STARTUP_GRACE_MS = 400;
    private static final long STALL_CONFIRM_MS = 300;
    private static final long REVERSE_BURST_MS = 40;
    public static final long HOPPER_LOADER_FORWARD_MS = 250;
    public static final long LOADER_FORWARD_OVERLAP_MS = 50;
    public static final long HOPPER_LOADER_REVERSE_MS = 75;
    public static final long LOADER_PULSE_ON_MS = 200;
    public static final long LOADER_RECOVERY_MS = 600; // Loader rests while the hopper feeds the next ball.
    public static final long LOADER_SPEED_WAIT_MAX_MS = 800;
    public static final long INTAKE_FEED_PAUSE_MS = 200;
    public static final boolean HOPPER_LOADER_FEED_CYCLE_ENABLED = true;
    private static final double HOPPER_LOADER_REVERSE_POWER = 0.3;
    private static final long LOADER_START_MS = HOPPER_LOADER_FORWARD_MS - LOADER_FORWARD_OVERLAP_MS;
    private static final long HOPPER_REVERSE_START_MS = HOPPER_LOADER_FORWARD_MS;
    private static final long HOPPER_REVERSE_END_MS = HOPPER_REVERSE_START_MS + HOPPER_LOADER_REVERSE_MS;
    private static final long LOADER_STOP_MS = LOADER_START_MS + LOADER_PULSE_ON_MS;
    public static final long HOPPER_LOADER_CYCLE_MS = LOADER_STOP_MS + LOADER_RECOVERY_MS;
    private boolean jamRecoveryEnabled;
    private boolean hopperStallDetectionEnabled = true;
    private boolean reversing;
    private boolean manualReverse;
    private long forwardStartTime;
    private long stallStartTime;
    private long reverseStartTime;
    private long hopperForwardStartTime;
    private long hopperStallStartTime;
    private double hopperPeakSpeed;
    private boolean hopperRequested;
    private boolean hopperStalled;
    private boolean hopperLoaderOverride;
    private boolean lastLoaderRunning;
    private long loaderFeedStartTime;
    private long loaderFeedPausedMs;
    private long lastStartedLoaderPulseCycle = -1;
    private long loaderPulseStartTime;
    private long loaderSpeedWaitStartTime;
    private FeedPhase feedPhase = FeedPhase.OFF;
    private long feedPhaseStartTime;
    private boolean hopperAfterReverseForwardEnabled = true;
    private boolean intakeEnabled = true;
    private boolean lastY = false;
    private boolean purgeRunning;

    private enum FeedPhase { OFF, FORWARD, WAITING_FOR_SPEED, LOADER_PULSE, RECOVERY }

    private void setFeedPhase(FeedPhase phase, long now) {
        if (feedPhase != phase) {
            feedPhase = phase;
            feedPhaseStartTime = now;
        }
    }

    public Intake(HardwareMap hardwareMap, DcMotor.Direction direction) {
        if (direction == null) {
            direction = DcMotor.Direction.FORWARD;
        }
        pickupMotor = hardwareMap.get(DcMotor.class, "Intake");
        pickupMotor.setDirection(direction);
        pickupMotor.setPower(0);
        intakeEncoder = pickupMotor instanceof DcMotorEx ? (DcMotorEx) pickupMotor : null;
        hopper = hardwareMap.tryGet(DcMotor.class, "Hopper");
        hopperEncoder = hopper instanceof DcMotorEx ? (DcMotorEx) hopper : null;
        if (hopper != null) {
            hopper.setDirection(DcMotor.Direction.FORWARD);
            hopper.setPower(0);
        }
    }

    public void update(Gamepad gamepad, Gamepad gamepad2) {
        updateEnabled(gamepad, gamepad2);
        if (CtrlAltDelOpMode.killMotors) {
            reversing = false;
            forwardStartTime = 0;
            stallStartTime = 0;
            setCollectionRunning(false);
            return;
        }
        if (purgeRunning) {
            applyPurge();
            return;
        }
        if (manualReverse) {
            reversing = false;
            forwardStartTime = 0;
            stallStartTime = 0;
            setCollectionRunning(intakeEnabled);
            return;
        }
        if (jamRecoveryEnabled) {
            updateJamRecovery();
        } else {
            setCollectionRunning(intakeEnabled);
        }
    }

    public void setPurgeRunning(boolean running) {
        if (purgeRunning == running) return;
        purgeRunning = running;
        onLoaderRunning(false);
        reversing = false;
        forwardStartTime = 0;
        stallStartTime = 0;
        hopperForwardStartTime = 0;
        hopperStallStartTime = 0;
        hopperPeakSpeed = 0;
        hopperStalled = false;
        if (running) applyPurge();
        else setCollectionRunning(false);
    }

    public boolean isPurgeRunning() {
        return purgeRunning;
    }

    private void applyPurge() {
        if (CtrlAltDelOpMode.killMotors) return;
        hopperRequested = false;
        pickupMotor.setPower(-INTAKE_POWER);
        if (hopper != null) hopper.setPower(-HOPPER_POWER);
    }

    public void enableJamRecovery() {
        jamRecoveryEnabled = true;
    }

    public void disableJamDetection() {
        jamRecoveryEnabled = false;
        hopperStallDetectionEnabled = false;
        reversing = false;
        hopperStalled = false;
        forwardStartTime = 0;
        stallStartTime = 0;
        hopperStallStartTime = 0;
    }

    public void updateEnabled(Gamepad gamepad, Gamepad gamepad2) {
        boolean yPressed = gamepad.y || gamepad2.y;

        // Detect fresh press only
        if (yPressed && !lastY) {
            intakeEnabled = !intakeEnabled;
            hopperLoaderOverride = false;
            reversing = false;
            forwardStartTime = 0;
            stallStartTime = 0;
            hopperStalled = false;
            hopperStallStartTime = 0;
            hopperForwardStartTime = 0;
            hopperPeakSpeed = 0;
        }
        lastY = yPressed;
    }

    public boolean isEnabled() {
        return intakeEnabled;
    }

    public boolean isReversing() {
        return reversing || manualReverse;
    }

    public void setManualReverse(boolean running) {
        manualReverse = running;
    }

    public boolean isManualReversing() {
        return manualReverse;
    }

    public boolean hasHopper() {
        return hopper != null;
    }

    public boolean isHopperStalled() {
        return hopperStalled;
    }

    public boolean isHopperLoaderOverride() {
        return hopperLoaderOverride;
    }

    public boolean isLoaderFeedRequested() {
        return lastLoaderRunning;
    }

    public void setHopperAfterReverseForwardEnabled(boolean enabled) {
        hopperAfterReverseForwardEnabled = enabled;
    }

    private boolean isAfterReverseForward(long phase) {
        return hopperAfterReverseForwardEnabled && phase >= HOPPER_REVERSE_END_MS;
    }

    public void runHopper(boolean running) {
        hopperRequested = running && !CtrlAltDelOpMode.killMotors;
        boolean reverseHopper = false;
        if (HOPPER_LOADER_FEED_CYCLE_ENABLED && hopperRequested && loaderFeedStartTime != 0) {
            long phase = getLoaderFeedElapsedMs() % HOPPER_LOADER_CYCLE_MS;
            hopperRequested = phase < HOPPER_LOADER_FORWARD_MS || isAfterReverseForward(phase);
            reverseHopper = phase >= HOPPER_REVERSE_START_MS && phase < HOPPER_REVERSE_END_MS;
        }
        if (feedPhase == FeedPhase.WAITING_FOR_SPEED) {
            hopperRequested = false;
            reverseHopper = false;
        }
        if (hopper == null) return;
        boolean shouldRun = hopperRequested && !hopperStalled;
        if (shouldRun && hopper.getPower() <= 0) {
            hopperForwardStartTime = System.currentTimeMillis();
            hopperStallStartTime = 0;
            hopperPeakSpeed = 0;
        }
        if (!shouldRun) hopperForwardStartTime = 0;
        hopper.setPower(shouldRun ? HOPPER_POWER : reverseHopper ? -HOPPER_LOADER_REVERSE_POWER : 0);
    }

    public void setCollectionRunning(boolean running) {
        pickupMotor.setPower(manualReverse && !CtrlAltDelOpMode.killMotors ? -INTAKE_POWER
                : running && !isIntakeFeedPauseActive() ? INTAKE_POWER : 0);
        runHopper(running || hopperLoaderOverride);
    }

    private void onLoaderRunning(boolean running) {
        if (CtrlAltDelOpMode.killMotors) {
            loaderFeedStartTime = 0;
            resetLoaderSpeedWait();
            hopperLoaderOverride = false;
            lastLoaderRunning = false;
            runHopper(false);
            return;
        }
        if (running) {
            if (!lastLoaderRunning) {
                intakeEnabled = true;
                loaderFeedStartTime = HOPPER_LOADER_FEED_CYCLE_ENABLED
                        ? System.currentTimeMillis() : 0;
                resetLoaderSpeedWait();
                if (HOPPER_LOADER_FEED_CYCLE_ENABLED) setFeedPhase(FeedPhase.FORWARD, loaderFeedStartTime);
                hopperStalled = false;
                hopperLoaderOverride = true;
                setCollectionRunning(true);
            } else {
                hopperLoaderOverride = true;
                runHopper(true);
            }
        } else if (lastLoaderRunning) {
            loaderFeedStartTime = 0;
            resetLoaderSpeedWait();
            hopperLoaderOverride = false;
            runHopper(intakeEnabled);
        }
        lastLoaderRunning = running;
    }

    private void resetLoaderSpeedWait() {
        lastStartedLoaderPulseCycle = -1;
        loaderPulseStartTime = 0;
        loaderSpeedWaitStartTime = 0;
        loaderFeedPausedMs = 0;
        setFeedPhase(FeedPhase.OFF, System.currentTimeMillis());
    }

    // Own the hopper timing, speed wait, and loader pulse as one feed cycle.
    public boolean updateLoaderFeed(boolean running, boolean speedReady) {
        onLoaderRunning(running);
        if (!running || CtrlAltDelOpMode.killMotors) return false;
        if (!HOPPER_LOADER_FEED_CYCLE_ENABLED) {
            if (!jamRecoveryEnabled) setCollectionRunning(true);
            return true;
        }
        updateLoaderSpeedGate(speedReady);
        // TeleOp's intake update owns jam recovery; Auto has no intake update loop.
        if (!jamRecoveryEnabled) setCollectionRunning(true);
        return isLoaderPulseActive();
    }

    private void updateLoaderSpeedGate(boolean speedReady) {
        if (loaderFeedStartTime == 0) return;
        long now = System.currentTimeMillis();
        long elapsed = getLoaderFeedElapsedMs();
        long cycle = elapsed / HOPPER_LOADER_CYCLE_MS;
        long phase = elapsed % HOPPER_LOADER_CYCLE_MS;
        if (cycle == lastStartedLoaderPulseCycle) {
            setFeedPhase(now - loaderPulseStartTime < LOADER_PULSE_ON_MS
                    ? FeedPhase.LOADER_PULSE : FeedPhase.RECOVERY, now);
            return;
        }
        if (phase < LOADER_START_MS && feedPhase != FeedPhase.WAITING_FOR_SPEED) {
            setFeedPhase(FeedPhase.FORWARD, now);
            return;
        }

        if (cycle == 0 || speedReady || loaderSpeedWaitStartTime != 0
                && now - loaderSpeedWaitStartTime >= LOADER_SPEED_WAIT_MAX_MS) {
            // Time the full pulse from the actual command, including after a late loop.
            lastStartedLoaderPulseCycle = cycle;
            loaderPulseStartTime = now;
            if (loaderSpeedWaitStartTime != 0) {
                loaderFeedPausedMs += now - loaderSpeedWaitStartTime;
                loaderFeedPausedMs--; // Advance from just before the pulse to its start.
            } else {
                // Account for a late update without shortening the loader pulse.
                loaderFeedPausedMs += phase - LOADER_START_MS;
            }
            loaderSpeedWaitStartTime = 0;
            setFeedPhase(FeedPhase.LOADER_PULSE, now);
            return;
        }

        if (loaderSpeedWaitStartTime == 0) {
            loaderSpeedWaitStartTime = now;
            // Pause the feed clock immediately before the next loader pulse.
            loaderFeedPausedMs += phase - (LOADER_START_MS - 1);
            setFeedPhase(FeedPhase.WAITING_FOR_SPEED, now);
        }
    }

    public long getLoaderFeedElapsedMs() {
        if (loaderFeedStartTime == 0) return 0;
        long now = System.currentTimeMillis();
        long currentWaitMs = loaderSpeedWaitStartTime == 0 ? 0 : now - loaderSpeedWaitStartTime;
        return now - loaderFeedStartTime - loaderFeedPausedMs - currentWaitMs;
    }

    private boolean isLoaderPulseActive() {
        if (CtrlAltDelOpMode.killMotors || !lastLoaderRunning || feedPhase == FeedPhase.WAITING_FOR_SPEED) return false;
        if (!HOPPER_LOADER_FEED_CYCLE_ENABLED) return true;
        return loaderPulseStartTime != 0
                && System.currentTimeMillis() - loaderPulseStartTime < LOADER_PULSE_ON_MS;
    }

    public boolean isIntakeFeedPauseActive() {
        if (feedPhase == FeedPhase.WAITING_FOR_SPEED) return true;
        if (!HOPPER_LOADER_FEED_CYCLE_ENABLED || !lastLoaderRunning || loaderFeedStartTime == 0) return false;
        long phase = getLoaderFeedElapsedMs() % HOPPER_LOADER_CYCLE_MS;
        return phase >= LOADER_STOP_MS && phase < LOADER_STOP_MS + INTAKE_FEED_PAUSE_MS;
    }

    public void updateHopperStall() {
        if (!hopperStallDetectionEnabled) return;

        long now = System.currentTimeMillis();
        if (hopperStalled) {
            if (now - hopperStallStartTime >= HOPPER_STALL_RESTART_MS) {
                hopperStalled = false;
                hopperStallStartTime = 0;
                hopperForwardStartTime = 0;
                hopperPeakSpeed = 0;
            }
            // Let the normal motor update restart it before checking speed again.
            return;
        }

        // The loader feed cycle handles spacing with a reverse and stop; do not latch off later forward phases.
        if (lastLoaderRunning) return;
        if (hopperEncoder == null || !hopperRequested || hopperStalled) return;
        if (now - hopperForwardStartTime < STARTUP_GRACE_MS) return;
        double speed = Math.abs(hopperEncoder.getVelocity());
        hopperPeakSpeed = Math.max(hopperPeakSpeed, speed);
        double slowdownLimit = Math.max(STALL_SPEED_TICKS_PER_SECOND,
                hopperPeakSpeed * HOPPER_SLOWDOWN_FRACTION);
        if (speed > slowdownLimit) {
            hopperStallStartTime = 0;
            return;
        }
        if (hopperStallStartTime == 0) hopperStallStartTime = now;
        if (now - hopperStallStartTime >= HOPPER_SLOWDOWN_CONFIRM_MS) {
            hopperStalled = true;
            hopperStallStartTime = now; // Measure the retry delay from when the motor actually stops.
            hopper.setPower(0);
        }
    }

    private void updateJamRecovery() {
        if (!intakeEnabled) {
            setCollectionRunning(false);
            forwardStartTime = 0;
            stallStartTime = 0;
            return;
        }

        if (isIntakeFeedPauseActive()) {
            reversing = false;
            forwardStartTime = 0;
            stallStartTime = 0;
            setCollectionRunning(true);
            return;
        }

        long now = System.currentTimeMillis();
        if (reversing) {
            if (now - reverseStartTime < REVERSE_BURST_MS) {
                pickupMotor.setPower(-INTAKE_POWER);
                runHopper(true);
                return;
            }
            reversing = false;
            forwardStartTime = now;
            stallStartTime = 0;
        }

        setCollectionRunning(true);
        if (intakeEncoder == null) return;
        if (forwardStartTime == 0) forwardStartTime = now;
        if (now - forwardStartTime < STARTUP_GRACE_MS) return;

        if (Math.abs(intakeEncoder.getVelocity()) > STALL_SPEED_TICKS_PER_SECOND) {
            stallStartTime = 0;
            return;
        }

        if (stallStartTime == 0) stallStartTime = now;
        if (now - stallStartTime < STALL_CONFIRM_MS) return;

        reversing = true;
        reverseStartTime = now;
        pickupMotor.setPower(-INTAKE_POWER);
        runHopper(true);
    }

    public void addTelemetry(Telemetry telemetry) {
        telemetry.addData("Intake power", "%.0f%%", pickupMotor.getPower() * 100);
        if (hopper != null) telemetry.addData("Hopper power", "%.0f%%", hopper.getPower() * 100);
        if (hopper != null) {
            telemetry.addData("Hopper speed", hopperEncoder == null ? "No encoder" :
                    String.format("%.0f ticks/s", hopperEncoder.getVelocity()));
            if (hopperEncoder != null && hopperStallDetectionEnabled) telemetry.addData("Hopper slowdown limit", "%.0f ticks/s",
                    Math.max(STALL_SPEED_TICKS_PER_SECOND, hopperPeakSpeed * HOPPER_SLOWDOWN_FRACTION));
            telemetry.addData("Hopper", !hopperStallDetectionEnabled ? "Jam detection disabled" :
                    hopperStalled ? "⚠ Stalled; automatic retry after " + HOPPER_STALL_RESTART_MS + " ms" : "✅ Clear");
        }
        String hopperPhase = "Off";
        if (lastLoaderRunning) {
            if (!HOPPER_LOADER_FEED_CYCLE_ENABLED) {
                hopperPhase = "Continuous";
            } else {
                long phase = getLoaderFeedElapsedMs() % HOPPER_LOADER_CYCLE_MS;
                hopperPhase = phase < HOPPER_LOADER_FORWARD_MS || isAfterReverseForward(phase) ? "Forward" :
                        phase >= HOPPER_REVERSE_START_MS && phase < HOPPER_REVERSE_END_MS ? "Reverse" : "Stopped";
            }
        }
        telemetry.addData("Hopper feed phase", hopperPhase);
        if (lastLoaderRunning && HOPPER_LOADER_FEED_CYCLE_ENABLED) {
            telemetry.addData("Loader feed phase", "%s (%d ms)", feedPhase,
                    System.currentTimeMillis() - feedPhaseStartTime);
        }
        if (jamRecoveryEnabled) {
            telemetry.addData("Intake Speed", intakeEncoder == null ? "No encoder" :
                    String.format("%.0f ticks/s", intakeEncoder.getVelocity()));
            telemetry.addData("Jam recovery", reversing ? "↩ Reversing" : "✅ Clear");
        }
        telemetry.addLine();
    }
}
