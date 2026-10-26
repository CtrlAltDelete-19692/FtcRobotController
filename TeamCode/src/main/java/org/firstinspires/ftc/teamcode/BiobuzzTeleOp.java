package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;


@TeleOp(name = "BiobuzzTeleOp")

public class BiobuzzTeleOp extends CtrlAltDelOpMode {

    public Drive drive;
    private RotatingLauncher launcher;
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
        launcher = new RotatingLauncher(hardwareMap, null, null);

        //led = hardwareMap.get(Servo.class, "led");
        //led.setPosition(ledPosition);

        // Start our timer after the OpMode starts.
        lastLoopTime = System.nanoTime();

        TelemetryDashboard dashboard = new TelemetryDashboard(telemetry);

        while (!isStarted()) {
            //if (gamepad1.x) changeTeam(true);
            //if (gamepad1.b) changeTeam(false);

            dashboard.addStatus(teamTagId, drive.getDriveMode(), killMotors);
            launcher.addTelemetry(telemetry);
            intake.addTelemetry(telemetry);
            drive.addTelemetry(telemetry);
            dashboard.update();
        }

        waitForStart();

        drive.imu.resetYaw();

        while (opModeIsActive()) {
            drive.update(gamepad1, gamepad2, teamTagId);
            intake.update(gamepad1, gamepad2);
            launcher.update(gamepad1, gamepad2);

            long currentTime = System.nanoTime();

            double deltaTime =
                    (currentTime - lastLoopTime) / 1_000_000_000.0;

            lastLoopTime = currentTime;

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

//            telemetry.addData(
//                    "LED Position",
//                    "%.3f",
//                    ledPosition
//            );


            dashboard.addStatus(teamTagId, drive.getDriveMode(), killMotors);
            launcher.addTelemetry(telemetry);
            intake.addTelemetry(telemetry);
            drive.addTelemetry(telemetry);
            dashboard.update();
        }
    }
}
