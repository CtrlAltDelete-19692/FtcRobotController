package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;

@Autonomous(name = "KingBobAutonomous")

public class KingBobAutonomous extends CtrlAltDelOpMode {
    private int teamTagId = 20;

    private Drive drive;

    GoBildaPinpointDriver pinpoint;

    @Override
    public void runOpMode() throws InterruptedException {

        pinpoint = hardwareMap.get(GoBildaPinpointDriver.class, "Odo");

        // 1. Initialize settings (Check user guide for your specific pod's ticks/mm)
        //pinpoint.setXOffset(-120.0); // Lateral pod offset
        //pinpoint.setYOffset(-120.0); // Forward pod offset
        pinpoint.setEncoderResolution(GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_4_BAR_POD);
        //pinpoint.setEncoderDirections(GoBildaPinpointDriver.EncoderDirection.FORWARD, GoBildaPinpointDriver.EncoderDirection.FORWARD);

        pinpoint.resetPosAndIMU(); // Sets current position to (0,0,0)

        drive = new Drive(hardwareMap, "King Bob", false);
        drive.setDriveMode(Drive.DriveMode.FIELD_CENTRIC);
        TelemetryDashboard dashboard = new TelemetryDashboard(telemetry, drive, null, null, null, null);

//        while (!isStarted()) {
//            dashboard.update(teamTagId);
//        }

        //int direction = -1; // We program our moves based on Red, and multiple by direction (-1) to reverse rotation / strafing for blue
        //teamTagId = 20;

        waitForStart();

        if (opModeIsActive()) {
            pinpoint.update();

            // Move into shooting position
            goToPose(-8, -8);
            turnToHeading(45);
            // shoot(3);
            sleep(3000);

            // Gather 3 new artifacts
            turnToHeading(90);
            goToPose(-12, -8); // Backup
            goToPose(-12, -12); // Strafe right
            goToPose(0, -12); // Move forward to collect artifacts
            sleep(3000);

            // Move into shooting position
            goToPose(-8, -8);
            turnToHeading(45);
            // shoot(3)
            sleep(3000);

            //dashboard.update(teamTagId);

            stopAllMotors();
        }

        while (opModeIsActive()) {
            pause(20000);
        }
    }

    private void driveForTime(double strafe, double forward, double rotate, double durationMs, int direction) {
        long start = System.currentTimeMillis();
        while (opModeIsActive() && System.currentTimeMillis() - start < durationMs) {
            pinpoint.update();

            telemetry.addData("X", pinpoint.getPosition().getX(DistanceUnit.INCH));
            telemetry.addData("Y", pinpoint.getPosition().getY(DistanceUnit.INCH));
            telemetry.update();

            drive.driveCommand(strafe * direction, forward, rotate * direction);

            idle();
        }

        driveStop();
    }

    private void goToPose(double targetX, double targetY) {
        final double buffer = 2.0; // inches
        final double minSpeed = 0.15;
        final double maxSpeed = 0.5;
        final double kP = 0.05; // proportional gain
        final long timeoutMs = 5000; // safety timeout

        long startTime = System.currentTimeMillis();

        while (opModeIsActive()) {
            // Safety timeout so we do not drive forever if odometry is wrong
            if (System.currentTimeMillis() - startTime > timeoutMs) {
                telemetry.addLine("goToPose timed out");
                telemetry.update();
                break;
            }

            // Get latest odometry
            pinpoint.update();
            Pose2D pos = pinpoint.getPosition();

            double currentX = pos.getX(DistanceUnit.INCH) * -1; // Flip since it's mounted "backwards"
            double currentY = pos.getY(DistanceUnit.INCH);

            // Error from target
            double xError = targetX - currentX;
            double yError = targetY - currentY;

            double strafe = 0;
            double forward = 0;

            int xDirection = xError > 0 ? 1 : -1;
            if (Math.abs(xError) > buffer) {
                strafe = Math.abs(xError) * kP;

                if (strafe < minSpeed) {
                    strafe = minSpeed;
                }
                if (strafe > maxSpeed) {
                    strafe = maxSpeed;
                }

                strafe *= xDirection;
            } else {
                strafe = 0;
            }

            int yDirection = yError > 0 ? 1 : -1;
            if (Math.abs(yError) > buffer) {
                forward = Math.abs(yError) * kP;

                if (forward < minSpeed) {
                    forward = minSpeed;
                }
                if (forward > maxSpeed) {
                    forward = maxSpeed;
                }

                forward *= yDirection;
            } else {
                forward = 0;
            }

            // Stop once both axes are within tolerance
            if (forward == 0 && strafe == 0) {
                break;
            }

            drive.driveCommand(strafe, forward, 0);

            telemetry.addLine(String.format("Target:  (%.1f, %.1f)", targetX, targetY));
            telemetry.addLine(String.format("Current: (%.1f, %.1f)", currentX, currentY));
            telemetry.addLine(String.format("Error:   (%.1f, %.1f)", xError, yError));
            telemetry.addLine(String.format("Drive:   S %.1f | F %.1f", strafe, forward));
            telemetry.update();

            idle();
        }

        driveStop();
    }

    private void turnToHeading(double targetHeading) {
        final double headingBuffer = 3.0; // degrees
        final double minTurn = 0.12;
        final double maxTurn = 0.35;
        final double headingKP = 0.01;
        final long timeoutMs = 4000;

        long startTime = System.currentTimeMillis();

        while (opModeIsActive()) {
            // Safety timeout
            if (System.currentTimeMillis() - startTime > timeoutMs) {
                telemetry.addLine("turnToHeading timed out");
                telemetry.update();
                break;
            }

            // Update odometry
            pinpoint.update();
            Pose2D pos = pinpoint.getPosition();

            double currentHeading = pos.getHeading(AngleUnit.DEGREES) * -1; // Flip

            // Normalize heading error to -180 to 180
            double headingError = targetHeading - currentHeading;

            while (headingError > 180) {
                headingError -= 360;
            }

            while (headingError < -180) {
                headingError += 360;
            }

            double turn = 0;

            int turnDirection = headingError > 0 ? 1 : -1;
            if (Math.abs(headingError) > headingBuffer) {
                turn = Math.abs(headingError) * headingKP;

                if (turn < minTurn) {
                    turn = minTurn;
                }

                if (turn > maxTurn) {
                    turn = maxTurn;
                }

                turn *= turnDirection;
            } else {
                turn = 0;
            }

            if (turn == 0) {
                break;
            }

            drive.driveCommand(0, 0, turn);

            telemetry.addLine(String.format("Target Heading:  %.1f", targetHeading));
            telemetry.addLine(String.format("Current Heading: %.1f", currentHeading));
            telemetry.addLine(String.format("Heading Error:   %.1f", headingError));
            telemetry.addLine(String.format("Turn:            %.2f", turn));
            telemetry.update();

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
