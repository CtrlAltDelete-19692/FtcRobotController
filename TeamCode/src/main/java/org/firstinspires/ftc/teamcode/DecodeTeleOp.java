package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@TeleOp(name = "DecodeTeleOp")

public class DecodeTeleOp extends LinearOpMode {

    private int teamTagId = 20; // TODO: Create function to update team tag Id
    private static final double INTAKE_POWER = 1.0; // Between 0 and 1
    private static final double LOADER_POWER = 0.5; // Between 0 and 1
    private static final double LAUNCHER_IDLE_RPM = 0;  // maintains spin
    private static final double LAUNCHER_FULL_RPM = 200;  // full shooting speed
    private static final double TRIGGER_DEADZONE = 0.05;

    @Override
    public void runOpMode() throws InterruptedException {
        telemetry.speak("Good luck!");

        Hardware hw = new Hardware();
        hw.setup(hardwareMap);

        Drive drive = new Drive();
        drive.setup(hw);

        TelemetryDashboard dashboard = new TelemetryDashboard(telemetry, hw);

        hw.limelight.start();

        waitForStart();
        while (opModeIsActive()) {
            if (!gamepad1.x) {
                drive.update(gamepad1);
            } else {
                drive.centerOnTag(teamTagId);
            }

            // Intake
            if (hw.intake != null) {
                if (gamepad2.left_trigger > TRIGGER_DEADZONE) {
                    hw.intake.setPower(INTAKE_POWER);
                } else {
                    hw.intake.setPower(0);
                }
            }

            // Loader
            if (hw.loader != null) {
                double currentLauncherVelocity = hw.launcher.getVelocity();
                boolean launcherReady = currentLauncherVelocity >= (LAUNCHER_FULL_RPM * 0.95); // Within 95% of FULL_RPM
                //            if (gamepad2.a && launcherReady) {
                if (gamepad2.a) {
                    hw.loader.setPower(LOADER_POWER);
                } else {
                    hw.loader.setPower(0);
                }
            }

            // Team color change
            if (gamepad1.y || gamepad2.y) { // TODO: Add logic to ensure it doesn't switch rapidly due to holding the button too long
                toggleTeamTag();
            }

            AprilTag aprilTag = null;
            if (hw.limelight != null) {
                aprilTag = new AprilTag(hw.limelight);
            }

            // Launcher
            double z = 3; // TODO: Update this default accordingly
            double x = 0;
            if (aprilTag != null) {
                z = aprilTag.getZ();
                x = aprilTag.getX();
            }
            double launcherVelocity = LAUNCHER_IDLE_RPM;
            if (gamepad2.right_trigger > TRIGGER_DEADZONE) {
                launcherVelocity = LAUNCHER_FULL_RPM;
                if (z <= 0.3) { // TODO: Tune accordingly, maybe set to linear relationship
                    launcherVelocity *= 0.5;
                } else if (z <= 0.6) {
                    launcherVelocity *= 0.7;
                }
            }
            hw.launcher.setVelocity(launcherVelocity);

            dashboard.update(teamTagId, drive, launcherVelocity, x, z);
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
