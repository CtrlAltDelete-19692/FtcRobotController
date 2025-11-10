package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@TeleOp(name = "DecodeTeleOp")

public class DecodeTeleOp extends LinearOpMode {

    private int teamTagId = 20; // TODO: Create function to update team tag Id
    private static final double INTAKE_POWER = 1.0; // Between 0 and 1
    private static final double TRIGGER_DEADZONE = 0.05;

    @Override
    public void runOpMode() throws InterruptedException {
        telemetry.speak("Good luck!");

        Hardware hw = new Hardware();
        hw.setup(hardwareMap);

        Drive drive = new Drive(hw);
        Launcher launcher = new Launcher(hw);
        Slides slides = new Slides(hw);

        TelemetryDashboard dashboard = new TelemetryDashboard(telemetry, hw);

        hw.limelight.start();

        waitForStart();
        while (opModeIsActive()) {
            drive.update(gamepad1, teamTagId);
            slides.update(gamepad2);
            launcher.update(gamepad2);

            // Team color change
            if (gamepad1.y || gamepad2.y) { // TODO: Add logic to ensure it doesn't switch rapidly due to holding the button too long
                toggleTeamTag();
            }

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

    public void toggleTeamTag() {
        if (teamTagId == 20) {
            teamTagId = 24;
        } else if (teamTagId == 24) {
            teamTagId = 20;
        }
    }
}
