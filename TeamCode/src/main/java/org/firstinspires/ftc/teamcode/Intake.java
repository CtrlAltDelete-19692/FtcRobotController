package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class Intake {

    public DcMotor pickupMotor;

    public static final double INTAKE_POWER = 0.8;  // Default power for pickup intake, 0 to 1
    private boolean intakeEnabled = true;
    private boolean lastY = false;

    public Intake(HardwareMap hardwareMap) {
        pickupMotor = hardwareMap.get(DcMotor.class, "PM");
        pickupMotor.setDirection(DcMotor.Direction.FORWARD);
        pickupMotor.setPower(0);
    }

    public void update(Gamepad gamepad, Gamepad gamepad2) {
        boolean yPressed = gamepad.y || gamepad2.y;
        boolean spinLauncher = gamepad2.right_trigger > CtrlAltDelOpMode.TRIGGER_DEADZONE;

        // Detect fresh press only
        if (yPressed && !lastY) {
            intakeEnabled = !intakeEnabled;
        }

        lastY = yPressed;

        // Apply power
        if (intakeEnabled || spinLauncher) {
            pickupMotor.setPower(INTAKE_POWER);
        } else {
            pickupMotor.setPower(0);
        }
    }
}