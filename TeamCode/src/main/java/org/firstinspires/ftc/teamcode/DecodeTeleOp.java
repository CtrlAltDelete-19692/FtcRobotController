package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.Telemetry;

@TeleOp(name = "DecodeTeleOp")

public class DecodeTeleOp extends CtrlAltDelOpMode {
    private Drive drive;
    private Launcher launcher;
    private AprilTag aprilTag;
    private Intake intake;
    private Slides slides;
    private Leds leds;

    private int teamTagId = 20;
    private int pipeline = 0;

    public static boolean oneController = false; // Set to true for testing with one controller, set false for competition

    private boolean xPressedLast = false;

    @Override
    public void runOpMode() throws InterruptedException {
        telemetry.speak("Good luck!");

        drive = new Drive(hardwareMap, null);
        aprilTag = new AprilTag(hardwareMap);
        aprilTag.pipelineSwitch(pipeline);
        launcher = new Launcher(hardwareMap, aprilTag);
        intake = new Intake(hardwareMap);
        slides = new Slides(hardwareMap);
        leds = new Leds(hardwareMap, aprilTag, launcher);

        TelemetryDashboard dashboard = new TelemetryDashboard(telemetry, drive, launcher, aprilTag, intake, slides);

        while (!isStarted()) {
            if (gamepad1.x) changeTeam(true);
            if (gamepad1.b) changeTeam(false);

            aprilTag.update();

            dashboard.update(teamTagId);
        }

        waitForStart();

        drive.imu.resetYaw();

        while (opModeIsActive()) {
            aprilTag.update();
            drive.update(gamepad1, gamepad2, teamTagId);
            slides.update(gamepad2);
            launcher.update(gamepad2, gamepad1);
            intake.update(gamepad1, gamepad2);
            leds.update();

            if (gamepad1.x) changeTeam(true);
            if (gamepad1.b) changeTeam(false);

            // Kill Motors for loading
            boolean xPressed = gamepad1.a;
            if (xPressed && !xPressedLast) {
                //toggleMotors();
            }
            xPressedLast = xPressed;

            // Telemetry
            dashboard.update(teamTagId);
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

        aprilTag.pipelineSwitch(pipeline);
    }
}
