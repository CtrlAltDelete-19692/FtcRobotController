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
    private static final long REVERSE_BURST_MS = 250;
    public static final long HOPPER_LOADER_INITIAL_PAUSE_MS = 600;
    private static final long HOPPER_LOADER_ON_MS = 200;
    private static final long HOPPER_LOADER_OFF_MS = 600;
    public static final boolean HOPPER_LOADER_FEED_CYCLE_ENABLED = true; // Pause, pulse, and reverse while loading.
    private static final long HOPPER_LOADER_REVERSE_MS = 75;
    private static final long HOPPER_LOADER_REVERSE_DELAY_MS = 20;
    private static final double HOPPER_LOADER_REVERSE_POWER = 0.3;
    public static final long HOPPER_LOADER_CYCLE_MS = HOPPER_LOADER_ON_MS + HOPPER_LOADER_OFF_MS;
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
    private long loaderFeedPauseTime;
    private boolean intakeEnabled = true;
    private boolean lastY = false;

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
        if (jamRecoveryEnabled) {
            updateJamRecovery();
        } else {
            spin(intakeEnabled);
        }
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

    public void runHopper(boolean running) {
        hopperRequested = running && !CtrlAltDelOpMode.killMotors;
        boolean reverseHopper = false;
        if (HOPPER_LOADER_FEED_CYCLE_ENABLED && hopperRequested && loaderFeedStartTime != 0) {
            long elapsed = System.currentTimeMillis() - loaderFeedStartTime;
            if (elapsed < HOPPER_LOADER_INITIAL_PAUSE_MS) {
                reverseHopper = elapsed >= HOPPER_LOADER_REVERSE_DELAY_MS
                        && elapsed < HOPPER_LOADER_REVERSE_DELAY_MS + HOPPER_LOADER_REVERSE_MS;
                hopperRequested = false;
            } else {
                long phase = (elapsed - HOPPER_LOADER_INITIAL_PAUSE_MS) % HOPPER_LOADER_CYCLE_MS;
                hopperRequested = phase < HOPPER_LOADER_ON_MS;
                reverseHopper = phase >= HOPPER_LOADER_ON_MS + HOPPER_LOADER_REVERSE_DELAY_MS
                        && phase < HOPPER_LOADER_ON_MS + HOPPER_LOADER_REVERSE_DELAY_MS
                        + HOPPER_LOADER_REVERSE_MS;
            }
        }
        if (hopper == null) return;
        boolean shouldRun = hopperRequested && !hopperStalled;
        if (shouldRun && hopper.getPower() <= 0) {
            hopperForwardStartTime = System.currentTimeMillis();
            hopperStallStartTime = 0;
            hopperPeakSpeed = 0;
        }
        if (!shouldRun) hopperForwardStartTime = 0;
        hopper.setPower(shouldRun ? HOPPER_POWER
                : reverseHopper && !hopperStalled
                        ? -HOPPER_LOADER_REVERSE_POWER : 0);
    }

    public void spin(boolean spinLauncher) {
        pickupMotor.setPower(spinLauncher ? INTAKE_POWER : 0);
        runHopper(spinLauncher || hopperLoaderOverride);
    }

    public void onLoaderRunning(boolean running) {
        if (!running) loaderFeedPauseTime = 0;
        if (CtrlAltDelOpMode.killMotors) {
            loaderFeedStartTime = 0;
            hopperLoaderOverride = false;
            lastLoaderRunning = false;
            runHopper(false);
            return;
        }
        if (running && hopper != null) {
            if (!lastLoaderRunning) {
                loaderFeedStartTime = HOPPER_LOADER_FEED_CYCLE_ENABLED
                        ? System.currentTimeMillis() : 0;
                hopperStalled = false;
            }
            hopperLoaderOverride = true;
            runHopper(true);
        } else if (lastLoaderRunning) {
            loaderFeedStartTime = 0;
            hopperLoaderOverride = false;
            runHopper(intakeEnabled);
        }
        lastLoaderRunning = running;
    }

    // A false value pauses an autonomous feed without ending/resetting its hopper cycle.
    // Finish the feed with onLoaderRunning(false) followed by spin(false).
    public void setAutonomousFeedRunning(boolean running) {
        long now = System.currentTimeMillis();
        if (CtrlAltDelOpMode.killMotors) {
            onLoaderRunning(false);
            spin(false);
            return;
        }
        if (running) {
            if (loaderFeedPauseTime != 0 && loaderFeedStartTime != 0) {
                loaderFeedStartTime += now - loaderFeedPauseTime;
            }
            loaderFeedPauseTime = 0;
            onLoaderRunning(true);
            spin(true);
        } else {
            if (lastLoaderRunning && loaderFeedPauseTime == 0) loaderFeedPauseTime = now;
            pickupMotor.setPower(0);
            runHopper(false);
        }
    }

    public void updateHopperStall() {
        if (!hopperStallDetectionEnabled) return;
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
                    hopperStalled ? "⚠ Stalled; run Loader to restart" : "✅ Clear");
        }
        if (jamRecoveryEnabled) {
            telemetry.addData("Intake Speed", intakeEncoder == null ? "No encoder" :
                    String.format("%.0f ticks/s", intakeEncoder.getVelocity()));
            telemetry.addData("Jam recovery", reversing ? "↩ Reversing" : "✅ Clear");
        }
        telemetry.addLine();
    }
}
