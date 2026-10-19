package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;


@TeleOp(name = "Biobuzz")

public class BiobuzzTeleOp extends CtrlAltDelOpMode {

    public Drive drive;

    //private DcMotor motor;
    private Intake intake;
    private Servo led;

    // Current LED position from 0.0 to 1.0
    private double ledPosition = 0.25;

    // How many seconds it takes to sweep through
    // the entire LED color range.
    private static final double FULL_COLOR_SWEEP_SECONDS = 10.0;

    // Used to calculate elapsed time between loops.
    private long lastLoopTime;

    private double power = 1;
    private boolean lastLb = false;
    private boolean lastRb = false;
    private int teamTagId = 20;


    @Override
    public void runOpMode() {
        drive = new Drive(hardwareMap, "Biobuzz", false);
        intake = new Intake(hardwareMap, DcMotor.Direction.REVERSE);

//        motor = hardwareMap.get(DcMotor.class, "motor");
//
//        // STUDENT CODE HERE
//        motor.setDirection(DcMotor.Direction.FORWARD);
//        motor.setPower(0);

        //led = hardwareMap.get(Servo.class, "led");

        // Start LED at the beginning of the color range.
        //led.setPosition(ledPosition);


        waitForStart();

        drive.imu.resetYaw();

        // Start our timer after the OpMode starts.
        lastLoopTime = System.nanoTime();

        //TelemetryDashboard dashboard = new TelemetryDashboard(telemetry, drive, null, null, intake, null);

        while (opModeIsActive()) {
            drive.update(gamepad1, gamepad2, teamTagId);
            intake.update(gamepad1, gamepad2);

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

//            boolean xPressed = gamepad1.x;
//            if (xPressed) {
//                motor.setPower(power);
//            } else {
//                motor.setPower(0);
//            }

            boolean lbPressed = gamepad1.left_bumper;
            if (lbPressed && !lastLb) {
                power -= 0.1;
            }
            lastLb = lbPressed;

            boolean rbPressed = gamepad1.right_bumper;
            if (rbPressed && !lastRb) {
                power += 0.1;
            }
            lastRb = rbPressed;


            if (power > 1) {
                power = 1;
            } else if (power < -1) {
                power = -1;
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

//            if (rightTrigger) {
//                ledPosition += colorChange;
//
//                // Wrap back around to the beginning.
//                if (ledPosition > 0.75) {
//                    ledPosition = 0.25;
//                }
//            }


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
            //led.setPosition(ledPosition);


            // ---------------------------------------------------------
            // TELEMETRY
            // ---------------------------------------------------------

//            telemetry.addData(
//                    "LED Position",
//                    "%.3f",
//                    ledPosition
//            );

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

//            telemetry.addData(
//                    "DC Motor Power",
//                    motor.getPower()
//            );

            telemetry.addData("LBM", "Power: %.2f | Pos: %d",drive.leftBackMotor.getPower(), drive.leftBackMotor.getCurrentPosition());
            telemetry.addData("LFM", "Power: %.2f | Pos: %d",drive.leftFrontMotor.getPower(), drive.leftFrontMotor.getCurrentPosition());
            telemetry.addData("RBM", "Power: %.2f | Pos: %d",drive.rightBackMotor.getPower(), drive.rightBackMotor.getCurrentPosition());
            telemetry.addData("RFM", "Power: %.2f | Pos: %d",drive.rightFrontMotor.getPower(), drive.rightFrontMotor.getCurrentPosition());

            telemetry.update();
        }
    }
}