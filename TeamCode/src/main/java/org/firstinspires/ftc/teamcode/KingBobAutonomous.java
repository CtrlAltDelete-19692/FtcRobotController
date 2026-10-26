package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;

//@Autonomous(name = "KingBobAutonomous")

public class KingBobAutonomous extends CtrlAltDelOpMode {
    private int teamTagId = 20;

    private Drive drive;

    @Override
    public void runOpMode() throws InterruptedException {

        drive = new Drive(hardwareMap, "King Bob", false);
        drive.setDriveMode(Drive.DriveMode.FIELD_CENTRIC);
        TelemetryDashboard dashboard = new TelemetryDashboard(telemetry);

//        while (!isStarted()) {
//            dashboard.addStatus(teamTagId, drive.getDriveMode(), killMotors);
//            drive.addTelemetry(telemetry);
//            dashboard.update();
//        }

        //int direction = -1; // We program our moves based on Red, and multiple by direction (-1) to reverse rotation / strafing for blue
        //teamTagId = 20;

        waitForStart();

        if (opModeIsActive()) {

//            // Move into shooting position
//            goToPose(-8, -8);
//            turnToHeading(45);
//            // shoot(3);
//            sleep(3000);
//
//            // Gather 3 new artifacts
//            turnToHeading(90);
//            goToPose(-12, -8); // Backup
//            goToPose(-12, -12); // Strafe right
//            goToPose(0, -12); // Move forward to collect artifacts
//            sleep(3000);
//
//            // Move into shooting position
//            goToPose(-8, -8);
//            turnToHeading(45);
//            // shoot(3)
//            sleep(3000);

            //dashboard.addStatus(teamTagId, drive.getDriveMode(), killMotors);
            //drive.addTelemetry(telemetry);
            //dashboard.update();

            stopAllMotors();
        }

        while (opModeIsActive()) {
            pause(20000);
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

    private void driveStop() {
        drive.driveCommand(0, 0, 0);
    }

    private void stopAllMotors() {
        driveStop();
    }

    private void pause(long durationMs) {
        long start = System.currentTimeMillis();

        while (opModeIsActive()) {
            long elapsedMs   = System.currentTimeMillis() - start;
            long remainingMs = durationMs - elapsedMs;

            if (remainingMs <= 0) {
                break;
            }

            idle();
        }
    }
}
