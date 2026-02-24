package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.Telemetry;

@TeleOp(name = "DecodeTeleOp")

public class DecodeTeleOp extends LinearOpMode {

    private Hardware hw;
    private int teamTagId = 20;
    private int pipeline = 0;
    public static boolean oneController = false; // Set to true for testing with one controller, set false for competition
    private boolean xPressedLast = false;
    private boolean rbPressedLast = false;

    @Override
    public void runOpMode() throws InterruptedException {
        telemetry.speak("Good luck!");

        hw = new Hardware();
        hw.setup(hardwareMap);

        hw.aprilTag.start();
        hw.limelight.pipelineSwitch(pipeline);

        Drive drive = new Drive(hw);
        Launcher launcher = new Launcher(hw);
        Slides slides = new Slides(hw);
        Leds leds = new Leds(hw);

        TelemetryDashboard dashboard = new TelemetryDashboard(telemetry, hw);

        while (!isStarted()) {
            if (gamepad1.x) changeTeam(true);
            if (gamepad1.b) changeTeam(false);

            hw.aprilTag.update();

            dashboard.update(teamTagId, drive, launcher);
        }

        waitForStart();

        hw.imu.resetYaw();

        while (opModeIsActive()) {
            hw.aprilTag.update();
            drive.update(gamepad1, gamepad2, teamTagId);
            slides.update(gamepad2);
            launcher.update(gamepad2, gamepad1);
            leds.update(launcher);

            if (gamepad1.x) changeTeam(true);
            if (gamepad1.b) changeTeam(false);

            // Reset heading
            boolean rbPressed = gamepad1.right_bumper;
            if (rbPressed && !rbPressedLast) {
                hw.imu.resetYaw();
            }
            rbPressedLast = rbPressed;

            // Kill Motors for loading
            boolean xPressed = gamepad1.a;
            if (xPressed && !xPressedLast) {
                //hw.toggleMotors();
            }
            xPressedLast = xPressed;

            // Intake
            if (hw.pickupMotor != null) {
                if (gamepad2.dpad_right || gamepad1.dpad_right) {
                    launcher.pickupIntake(Launcher.INTAKE_POWER);
                }
                else {
                    launcher.pickupIntake(0);
                }
            }

            // Telemetry
            dashboard.update(teamTagId, drive, launcher);
            idle();
        }
    }

    public void changeTeam(boolean isBlue) {
        if (isBlue) {
            pipeline = 0; // Blue
            teamTagId = 20;
            telemetry.speak("Blue team");
        } else {
            pipeline = 1; // Red
            teamTagId = 24;
            telemetry.speak("Red team");
        }

        hw.limelight.pipelineSwitch(pipeline);
    }
}
