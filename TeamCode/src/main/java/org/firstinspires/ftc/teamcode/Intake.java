package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class Intake {

    public DcMotor pickupMotor;

    public static final double INTAKE_POWER = 1.0;  // Default power for pickup intake, 0 to 1

    public Intake(HardwareMap hardwareMap) {
        pickupMotor = hardwareMap.get(DcMotor.class, "PM");
        pickupMotor.setDirection(DcMotor.Direction.FORWARD);
    }

    public void update(Gamepad gamepad, Gamepad gamepad2) {
        if (gamepad.dpad_right || gamepad2.dpad_right || gamepad.y || gamepad2.y) {
            pickupMotor.setPower(INTAKE_POWER);
        }
        else {
            pickupMotor.setPower(0);
        }
    }
}
