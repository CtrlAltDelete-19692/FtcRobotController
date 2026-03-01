package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@TeleOp(name = "KingBobTeleOp")

public class KingBobTeleOp extends CtrlAltDelOpMode {
    public Drive drive;
    private int teamTagId = 20;

    @Override
    public void runOpMode() throws InterruptedException {
        telemetry.speak("Good luck!");

        drive = new Drive(hardwareMap, "King Bob");

        TelemetryDashboard dashboard = new TelemetryDashboard(telemetry, drive, null, null, null, null);

        waitForStart();

        drive.imu.resetYaw();

        while (opModeIsActive()) {
            drive.update(gamepad1, gamepad2, teamTagId);

            // Telemetry
            dashboard.update(teamTagId);
            idle();
        }
    }
}
