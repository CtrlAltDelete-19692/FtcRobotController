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

    public static final double INTAKE_POWER = 1;  // Default power for pickup intake, 0 to 1
    public static final double HOPPER_POWER = 0.6; // Default power for hopper, 0 to 1
    private static final double STALL_SPEED_TICKS_PER_SECOND = 5;
    private static final long STARTUP_GRACE_MS = 400;
    private static final long STALL_CONFIRM_MS = 150;
    private static final long REVERSE_BURST_MS = 250;
    private boolean jamRecoveryEnabled;
    private boolean reversing;
    private long forwardStartTime;
    private long stallStartTime;
    private long reverseStartTime;
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
        if (hopper != null) {
            hopper.setDirection(DcMotor.Direction.REVERSE);
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

    public void runHopper(boolean running) {
        if (hopper != null) hopper.setPower(running && !CtrlAltDelOpMode.killMotors ? HOPPER_POWER : 0);
    }

    public void spin(boolean spinLauncher) {
        pickupMotor.setPower(spinLauncher ? INTAKE_POWER : 0);
        if (hopper != null) hopper.setPower(spinLauncher ? HOPPER_POWER : 0);
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
                if (hopper != null) hopper.setPower(HOPPER_POWER);
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
        if (hopper != null) hopper.setPower(HOPPER_POWER);
    }

    public void addTelemetry(Telemetry telemetry) {
        telemetry.addData("Intake power", "%.0f%%", pickupMotor.getPower() * 100);
        if (hopper != null) telemetry.addData("Hopper power", "%.0f%%", hopper.getPower() * 100);
        if (jamRecoveryEnabled) {
            telemetry.addData("Intake Speed", intakeEncoder == null ? "No encoder" :
                    String.format("%.0f ticks/s", intakeEncoder.getVelocity()));
            telemetry.addData("Jam recovery", reversing ? "↩ Reversing" : "✅ Clear");
        }
        telemetry.addLine();
    }
}
