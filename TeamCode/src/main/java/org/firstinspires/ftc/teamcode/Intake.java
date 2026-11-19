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

    public static final double INTAKE_POWER = 0.5;  // Default power for pickup intake, 0 to 1
    public static final double HOPPER_POWER = 0.6; // Default power for hopper, 0 to 1
    private static final double STALL_SPEED_TICKS_PER_SECOND = 5;
    public static final double HOPPER_SLOWDOWN_FRACTION = 0.35; // Stop below 35% of this run's peak speed.
    private static final long HOPPER_SLOWDOWN_CONFIRM_MS = 250;
    private static final long STARTUP_GRACE_MS = 400;
    private static final long STALL_CONFIRM_MS = 150;
    private static final long REVERSE_BURST_MS = 200;
    public static final long HOPPER_LOADER_FORWARD_MS = 250;
    public static final long LOADER_FORWARD_OVERLAP_MS = 100;
    public static final long HOPPER_LOADER_REVERSE_MS = 125;
    public static final long LOADER_PULSE_ON_MS = 250;
    public static final long LOADER_RECOVERY_MS = 850; // Loader rests while the hopper feeds the next ball.
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
    private boolean hopperAfterReverseForwardEnabled = true;
    private boolean intakeEnabled = true;
    private boolean lastY = false;
    private boolean purgeRunning;

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
            spin(false);
            return;
        }
        if (purgeRunning) {
            applyPurge();
            return;
        }
        if (jamRecoveryEnabled) {
            updateJamRecovery();
        } else {
            spin(intakeEnabled);
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
        else spin(false);
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
        return reversing;
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
            long phase = (System.currentTimeMillis() - loaderFeedStartTime) % HOPPER_LOADER_CYCLE_MS;
            hopperRequested = phase < HOPPER_LOADER_FORWARD_MS || isAfterReverseForward(phase);
            reverseHopper = phase >= HOPPER_REVERSE_START_MS && phase < HOPPER_REVERSE_END_MS;
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

    public void spin(boolean spinLauncher) {
        pickupMotor.setPower(spinLauncher && !isIntakeFeedPauseActive() ? INTAKE_POWER : 0);
        runHopper(spinLauncher || hopperLoaderOverride);
    }

    public void onLoaderRunning(boolean running) {
        if (CtrlAltDelOpMode.killMotors) {
            loaderFeedStartTime = 0;
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
                hopperStalled = false;
                hopperLoaderOverride = true;
                spin(true);
            } else {
                hopperLoaderOverride = true;
                runHopper(true);
            }
        } else if (lastLoaderRunning) {
            loaderFeedStartTime = 0;
            hopperLoaderOverride = false;
            runHopper(intakeEnabled);
        }
        lastLoaderRunning = running;
    }

    public boolean isLoaderPulseActive() {
        if (CtrlAltDelOpMode.killMotors || !lastLoaderRunning) return false;
        if (!HOPPER_LOADER_FEED_CYCLE_ENABLED) return true;
        long phase = (System.currentTimeMillis() - loaderFeedStartTime) % HOPPER_LOADER_CYCLE_MS;
        return phase >= LOADER_START_MS && phase < LOADER_STOP_MS;
    }

    public boolean isIntakeFeedPauseActive() {
        if (!HOPPER_LOADER_FEED_CYCLE_ENABLED || !lastLoaderRunning || loaderFeedStartTime == 0) return false;
        long phase = (System.currentTimeMillis() - loaderFeedStartTime) % HOPPER_LOADER_CYCLE_MS;
        return phase >= LOADER_STOP_MS && phase < LOADER_STOP_MS + INTAKE_FEED_PAUSE_MS;
    }

    public void setAutonomousFeedRunning(boolean running) {
        onLoaderRunning(running);
        if (CtrlAltDelOpMode.killMotors) {
            onLoaderRunning(false);
            spin(false);
            return;
        }
        if (running) {
            spin(true);
        } else {
            pickupMotor.setPower(0);
            runHopper(false);
        }
    }

    public void updateHopperStall() {
        if (!hopperStallDetectionEnabled) return;
        // The loader feed cycle handles spacing with a reverse and stop; do not latch off later forward phases.
        if (lastLoaderRunning) return;
        if (hopperEncoder == null || !hopperRequested || hopperStalled) return;
        long now = System.currentTimeMillis();
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
            hopper.setPower(0);
        }
    }

    private void updateJamRecovery() {
        if (!intakeEnabled) {
            spin(false);
            forwardStartTime = 0;
            stallStartTime = 0;
            return;
        }

        if (isIntakeFeedPauseActive()) {
            reversing = false;
            forwardStartTime = 0;
            stallStartTime = 0;
            spin(true);
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

        spin(true);
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
                    hopperStalled ? "⚠ Stalled; toggle Y or run Loader to restart" : "✅ Clear");
        }
        String hopperPhase = "Off";
        if (lastLoaderRunning) {
            if (!HOPPER_LOADER_FEED_CYCLE_ENABLED) {
                hopperPhase = "Continuous";
            } else {
                long phase = (System.currentTimeMillis() - loaderFeedStartTime) % HOPPER_LOADER_CYCLE_MS;
                hopperPhase = phase < HOPPER_LOADER_FORWARD_MS || isAfterReverseForward(phase) ? "Forward" :
                        phase >= HOPPER_REVERSE_START_MS && phase < HOPPER_REVERSE_END_MS ? "Reverse" : "Stopped";
            }
        }
        telemetry.addData("Hopper feed phase", hopperPhase);
        if (jamRecoveryEnabled) {
            telemetry.addData("Intake Speed", intakeEncoder == null ? "No encoder" :
                    String.format("%.0f ticks/s", intakeEncoder.getVelocity()));
            telemetry.addData("Jam recovery", reversing ? "↩ Reversing" : "✅ Clear");
        }
        telemetry.addLine();
    }
}
