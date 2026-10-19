package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;


@TeleOp(name = "Turret")

public class Turret extends CtrlAltDelOpMode {

    public Drive drive;

    private DcMotor motor;
    private DcMotor turret;
    private Servo led;

    // Current LED position from 0.0 to 1.0
    private double ledPosition = 0.25;

    // How many seconds it takes to sweep through
    // the entire LED color range.
    private static final double FULL_COLOR_SWEEP_SECONDS = 10.0;

    // Used to calculate elapsed time between loops.
    private long lastLoopTime;

    private int teamTagId = 20;


    @Override
    public void runOpMode() {

        drive = new Drive(hardwareMap, "King Bob", false);

        motor = hardwareMap.get(DcMotor.class, "motor");
        turret = hardwareMap.get(DcMotor.class, "turret");

        // STUDENT CODE HERE
        motor.setDirection(DcMotor.Direction.FORWARD);
        motor.setPower(0);

        turret.setDirection(DcMotor.Direction.REVERSE);
        turret.setPower(0);

        led = hardwareMap.get(Servo.class, "led");

        // Start LED at the beginning of the color range.
        led.setPosition(ledPosition);


        waitForStart();

        drive.imu.resetYaw();

        // Start our timer after the OpMode starts.
        lastLoopTime = System.nanoTime();


        while (opModeIsActive()) {

            drive.update(gamepad1, gamepad2, teamTagId);


            // ---------------------------------------------------------
            // CALCULATE ELAPSED TIME
            // ---------------------------------------------------------

            long currentTime = System.nanoTime();

            double deltaTime =
                    (currentTime - lastLoopTime) / 1_000_000_000.0;

            lastLoopTime = currentTime;


            // ---------------------------------------------------------
            // DC MOTOR
            //
            // X = run motor
            // ---------------------------------------------------------

            boolean xPressed = gamepad1.x;
            if (xPressed) {
                motor.setPower(1);
            } else {
                motor.setPower(0);
            }

            boolean aPressed = gamepad1.a;
            boolean bPressed = gamepad1.b;
            if (aPressed) {
                turret.setPower(1);
            } else if (bPressed) {
                turret.setPower(-1);
            } else {
                turret.setPower(0);
            }


            // ---------------------------------------------------------
            // LED
            //
            // R2 = smoothly move forward through colors
            // L2 = smoothly move backward through colors
            // ---------------------------------------------------------

            boolean leftTrigger = gamepad1.left_trigger > 0.05;
            boolean rightTrigger = gamepad1.right_trigger > 0.05;


            // Amount the LED position should move this loop.
            double colorChange =
                    deltaTime / FULL_COLOR_SWEEP_SECONDS;


            // ---------------------------------------------------------
            // R2 = FORWARD THROUGH COLORS
            // ---------------------------------------------------------

            if (rightTrigger) {

                ledPosition += colorChange;

                // Wrap back around to the beginning.
                if (ledPosition > 0.75) {
                    ledPosition = 0.25;
                }
            }


            // ---------------------------------------------------------
            // L2 = BACKWARD THROUGH COLORS
            // ---------------------------------------------------------

            if (leftTrigger) {

                ledPosition -= colorChange;

                // Wrap around to the end.
                if (ledPosition < 0.25) {
                    ledPosition = 0.75;
                }
            }


            // Send the new position to the LED.
            led.setPosition(ledPosition);


            // ---------------------------------------------------------
            // TELEMETRY
            // ---------------------------------------------------------

            telemetry.addData(
                    "LED Position",
                    "%.3f",
                    ledPosition
            );

            telemetry.addData(
                    "L2",
                    "%.2f",
                    gamepad1.left_trigger
            );

            telemetry.addData(
                    "R2",
                    "%.2f",
                    gamepad1.right_trigger
            );

            telemetry.addData(
                    "DC Motor Power",
                    turret.getPower()
            );

            telemetry.update();
        }
    }
}