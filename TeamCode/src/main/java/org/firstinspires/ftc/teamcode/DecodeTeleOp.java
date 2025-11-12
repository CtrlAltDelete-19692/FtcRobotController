package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@TeleOp(name = "DecodeTeleOp")

public class DecodeTeleOp extends LinearOpMode {

    private Hardware hw;
    private int teamTagId = 20; // TODO: Create function to update team tag Id
    private int pipeline = 0;
    private static final double INTAKE_POWER = 1.0; // Between 0 and 1
    private static final double TRIGGER_DEADZONE = 0.05;

    private boolean yPressedLast = false;

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

        TelemetryDashboard dashboard = new TelemetryDashboard(telemetry, hw);

        waitForStart();
        while (opModeIsActive()) {
            hw.aprilTag.update();
            drive.update(gamepad1);
            slides.update(gamepad2);
            launcher.update(gamepad2);

            // Team color change
            boolean yPressed = gamepad1.y;
            if (yPressed && !yPressedLast) {
                toggleTeam();
            }
            yPressedLast = yPressed;

            // Intake (not in use)
            if (hw.intake != null) {
                if (gamepad2.left_trigger > TRIGGER_DEADZONE) {
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

    public void toggleTeam() {
        if (pipeline == 0) {
            pipeline = 1; // Red
            teamTagId = 24;
        } else if (pipeline == 1) {
            pipeline = 0; // Blue
            teamTagId = 20;
        }

        hw.limelight.pipelineSwitch(pipeline);
    }
}
