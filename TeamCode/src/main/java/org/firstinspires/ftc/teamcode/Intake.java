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
    public static final double HOPPER_POWER = 0.6; // Default power for hopper, 0 to 1
    private static final double STALL_SPEED_TICKS_PER_SECOND = 5;
    public static final double HOPPER_SLOWDOWN_FRACTION = 0.35; // Stop below 35% of this run's peak speed.
    private static final long HOPPER_SLOWDOWN_CONFIRM_MS = 250;
    private static final long STARTUP_GRACE_MS = 400;
    private static final long STALL_CONFIRM_MS = 150;
    private static final long REVERSE_BURST_MS = 250;
    private boolean jamRecoveryEnabled;
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
        if (hopper == null) return;
        boolean shouldRun = hopperRequested && !hopperStalled;
        if (shouldRun && hopper.getPower() == 0) {
            hopperForwardStartTime = System.currentTimeMillis();
            hopperStallStartTime = 0;
            hopperPeakSpeed = 0;
        }
        if (!shouldRun) hopperForwardStartTime = 0;
        hopper.setPower(shouldRun ? HOPPER_POWER : 0);
    }

    public void spin(boolean spinLauncher) {
        pickupMotor.setPower(spinLauncher ? INTAKE_POWER : 0);
        runHopper(spinLauncher || hopperLoaderOverride);
    }

    public void onLoaderRunning(boolean running) {
        if (running && !lastLoaderRunning && hopper != null && hopper.getPower() == 0
                && !CtrlAltDelOpMode.killMotors) {
            hopperLoaderOverride = true;
            hopperStalled = false;
            runHopper(true);
        }
        lastLoaderRunning = running;
    }

    public void updateHopperStall() {
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
            if (hopperEncoder != null) telemetry.addData("Hopper slowdown limit", "%.0f ticks/s",
                    Math.max(STALL_SPEED_TICKS_PER_SECOND, hopperPeakSpeed * HOPPER_SLOWDOWN_FRACTION));
            telemetry.addData("Hopper", hopperStalled ? "⚠ Stalled; run Loader to restart" : "✅ Clear");
        }
        if (jamRecoveryEnabled) {
            telemetry.addData("Intake Speed", intakeEncoder == null ? "No encoder" :
                    String.format("%.0f ticks/s", intakeEncoder.getVelocity()));
            telemetry.addData("Jam recovery", reversing ? "↩ Reversing" : "✅ Clear");
        }
        telemetry.addLine();
    }
}
