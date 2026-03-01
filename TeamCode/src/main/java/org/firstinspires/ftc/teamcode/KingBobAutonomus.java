package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;

@Autonomous(name = "KingBobAutonomous")

public class KingBobAutonomus extends CtrlAltDelOpMode {
    private int teamTagId = 20;

    private Drive drive;

    GoBildaPinpointDriver pinpoint;

    @Override
    public void runOpMode() throws InterruptedException {

        pinpoint = hardwareMap.get(GoBildaPinpointDriver.class, "Odo");

        // 1. Initialize settings (Check user guide for your specific pod's ticks/mm)
        //pinpoint.setXOffset(-120.0); // Lateral pod offset
        //pinpoint.setYOffset(-120.0); // Forward pod offset
        //pinpoint.setEncoderResolution(GoBildaPinpointDriver.EncoderResolution.goBILDA_4_BAR_POD);
        //pinpoint.setEncoderDirections(GoBildaPinpointDriver.EncoderDirection.FORWARD, GoBildaPinpointDriver.EncoderDirection.FORWARD);

        pinpoint.resetPosAndIMU(); // Sets current position to (0,0,0)

        TelemetryDashboard dashboard = new TelemetryDashboard(telemetry, drive, null, null, null, null);

        while (!isStarted()) {
            dashboard.update(teamTagId);
        }

        int direction = -1; // We program our moves based on Red, and multiple by direction (-1) to reverse rotation / strafing for blue
        teamTagId = 20;
        drive = new Drive(hardwareMap, "King Bob");

        waitForStart();


        if (opModeIsActive()) {
            pinpoint.update(); // Important: Call this every loop to refresh data

            driveForward(415, direction); // Roughly 1ft

            stopAllMotors();

            Pose2D pos = pinpoint.getPosition();
            telemetry.addData("X", pos.getX(DistanceUnit.INCH));
            telemetry.addData("Y", pos.getY(DistanceUnit.INCH));
            telemetry.addData("Heading", pos.getHeading(AngleUnit.DEGREES));

            dashboard.update(teamTagId);
        }
    }

    private void driveForTime(double strafe, double forward, double rotate, double durationMs, int direction) {
        long start = System.currentTimeMillis();
        while (opModeIsActive() && System.currentTimeMillis() - start < durationMs) {
            drive.driveCommand(strafe * direction, forward, rotate * direction);
            idle();
        }

        driveStop();
    }

    private void driveForward(double durationMs, int direction) {
        driveForTime(0, 1, 0, durationMs, direction);
    }

    private void driveStop() {
        drive.driveCommand(0, 0, 0);
    }

    private void stopAllMotors() {
        driveStop();
    }
}
