package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@TeleOp(name = "KingBobTeleOp")

public class KingBobTeleOp extends LinearOpMode {

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

        Drive drive = new Drive(hw, hardwareMap, "King Bob");

        TelemetryDashboard dashboard = new TelemetryDashboard(telemetry, hw);

        waitForStart();

        hw.imu.resetYaw();

        while (opModeIsActive()) {
            drive.update(gamepad1, gamepad2, teamTagId);

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

            // Telemetry
            dashboard.update(teamTagId, drive, null, null, null);
            idle();
        }
    }
}
