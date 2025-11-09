package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@TeleOp(name = "DecodeTeleOp")

public class DecodeTeleOp extends LinearOpMode {

    private int teamTagId = 21; // TODO: Create function to update team tag Id
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

        waitForStart();
        while (opModeIsActive()) {
            drive.update(gamepad1);

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

            // Launcher
//            AprilTag aprilTag = new AprilTag(hw.limelight);
//            double distance = aprilTag.getDistance(teamTagId);
            double launcherVelocity = LAUNCHER_IDLE_RPM;
            double distance = 1;
            if (gamepad2.right_trigger > TRIGGER_DEADZONE) {
                launcherVelocity = LAUNCHER_FULL_RPM;
                if (distance <= 1) {
                    launcherVelocity *= 0.8;
                } else if (distance <= 2) {
                    launcherVelocity *= 0.9;
                }
            }
            hw.launcher.setVelocity(launcherVelocity);

            dashboard.update(drive, launcherVelocity, distance);
            idle();
        }
    }
}
