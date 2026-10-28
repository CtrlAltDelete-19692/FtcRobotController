package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;

public class Intake {

    public DcMotor pickupMotor;

    public static final double INTAKE_POWER = 0.8;  // Default power for pickup intake, 0 to 1
    private boolean intakeEnabled = true;
    private boolean lastY = false;

    public Intake(HardwareMap hardwareMap, DcMotor.Direction direction) {
        if (direction == null) {
            direction = DcMotor.Direction.FORWARD;
        }
        pickupMotor = hardwareMap.get(DcMotor.class, "PM");
        pickupMotor.setDirection(direction);
        pickupMotor.setPower(0);
    }

    public void update(Gamepad gamepad, Gamepad gamepad2) {
        boolean yPressed = gamepad.y || gamepad2.y;

        // Detect fresh press only
        if (yPressed && !lastY) {
            intakeEnabled = !intakeEnabled;
        }
        lastY = yPressed;

        // boolean spinLauncher = gamepad2.right_trigger > CtrlAltDelOpMode.TRIGGER_DEADZONE || intakeEnabled;
        boolean spinLauncher = intakeEnabled;

        spin(spinLauncher);
    }

    public void spin(boolean spinLauncher) {
        if (spinLauncher) {
            pickupMotor.setPower(INTAKE_POWER);
        } else {
            pickupMotor.setPower(0);
        }
    }

    public void addTelemetry(Telemetry telemetry) {
        telemetry.addData("Pickup", "%.1f%%", pickupMotor.getPower() * 100);
        telemetry.addLine();
    }
}
