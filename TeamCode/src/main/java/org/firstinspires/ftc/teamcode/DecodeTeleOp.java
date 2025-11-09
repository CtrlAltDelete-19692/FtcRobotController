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
        telemetry.speak("Good luck! ");

        Hardware hw = new Hardware();
        hw.setup(hardwareMap);

        Drive drive = new Drive();
        drive.setup(hw);

        TelemetryDashboard dashboard = new TelemetryDashboard(telemetry, hw);

        waitForStart();
        while (opModeIsActive()) {
            drive.update(gamepad1);

            // Intake
//            if (gamepad2.left_trigger > TRIGGER_DEADZONE) {
//                hw.IntakeServo.setPower(INTAKE_POWER);
//            } else {
//                hw.IntakeServo.setPower(0);
//            }

            // Loader
            double currentLauncherVelocity = hw.Launcher.getVelocity();
            boolean launcherReady = currentLauncherVelocity >= (LAUNCHER_FULL_RPM * 0.95); // Within 95% of FULL_RPM
//            if (gamepad2.a && launcherReady) {
            if (gamepad2.a) {
                hw.LoaderServo.setPower(LOADER_POWER);
            } else {
                hw.LoaderServo.setPower(0);
            }

            // Launcher
//            AprilTag aprilTag = new AprilTag(hw.limelight);
//            double distance = aprilTag.getDistance(teamTagId);
              double launcherSpeed = LAUNCHER_IDLE_RPM;
//            if (distance <= 1) {
//                launcherSpeed = 4000;
//            } else if (distance <= 2) {
//                launcherSpeed = 4500;
//            }
            if (gamepad2.right_trigger > TRIGGER_DEADZONE) {
                launcherSpeed = LAUNCHER_FULL_RPM;
            }
            hw.Launcher.setVelocity(launcherSpeed);

            dashboard.update(drive, launcherSpeed, 1);
//            dashboard.update(drive, distance);
            idle();
        }
    }
}
