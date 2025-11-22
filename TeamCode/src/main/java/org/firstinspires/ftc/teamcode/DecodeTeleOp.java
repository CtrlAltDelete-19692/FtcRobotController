package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@TeleOp(name = "DecodeTeleOp")

public class DecodeTeleOp extends LinearOpMode {

    private Hardware hw;
    private int teamTagId = 20;
    private int pipeline = 0;
    public static boolean oneController = false; // Set to true for testing with one controller, set false for competition
    private static final double INTAKE_POWER = 1.0; // Between 0 and 1
    private boolean yPressedLast = false;
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
            leds.update(launcher, gamepad2);

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
                hw.toggleMotors();
            }
            xPressedLast = xPressed;

            // Intake (not in use)
            if (hw.intake != null) {
                if (gamepad2.left_trigger > Hardware.TRIGGER_DEADZONE) {
                    hw.intake.setPower(INTAKE_POWER);
                } else {
                    hw.intake.setPower(0);
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
