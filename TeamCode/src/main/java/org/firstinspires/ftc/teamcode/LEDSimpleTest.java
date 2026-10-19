package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;

@TeleOp(name = "LED Simple Test")
public class LEDSimpleTest extends LinearOpMode {

    private Servo led;

    @Override
    public void runOpMode() {

        led = hardwareMap.get(Servo.class, "led");

        waitForStart();

        while (opModeIsActive()) {

            led.setPosition(gamepad1.right_trigger);

            telemetry.addData("Position", gamepad1.right_trigger);
            telemetry.update();
        }
    }
}