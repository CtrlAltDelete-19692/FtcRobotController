package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

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
        TelemetryDashboard dashboard = new TelemetryDashboard(telemetry, drive, null, null, null, null);

        while (!isStarted()) {
            dashboard.update(teamTagId);
        }

        //int direction = -1; // We program our moves based on Red, and multiple by direction (-1) to reverse rotation / strafing for blue
        //teamTagId = 20;

        waitForStart();

        if (opModeIsActive()) {
            pinpoint.update();

            driveToPosition(12, 12);

            driveToPosition(0, -12);

            driveToPosition(-12, 0);

            dashboard.update(teamTagId);

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

    private void driveToPosition(double targetX, double targetY) {
        final double buffer = 2.0; // inches
        final double minSpeed = 0.15;
        final double maxSpeed = 1.0;
        final double kP = 0.05; // proportional gain
        final long timeoutMs = 4000; // safety timeout

        long startTime = System.currentTimeMillis();

        while (opModeIsActive()) {
            // Safety timeout so we do not drive forever if odometry is wrong
            if (System.currentTimeMillis() - startTime > timeoutMs) {
                telemetry.addLine("driveToPosition timed out");
                telemetry.update();
                break;
            }

            // Get latest odometry
            pinpoint.update();
            Pose2D pos = pinpoint.getPosition();

            double currentX = pos.getX(DistanceUnit.INCH);
            double currentY = pos.getY(DistanceUnit.INCH);

            // Error from target
            double xError = targetX - currentX;
            double yError = targetY - currentY;

            double forward = 0;
            double strafe = 0;

            // Proportional forward control with deadband and min/max speed clamp
            if (Math.abs(xError) > buffer) {
                double commandedSpeed = Math.abs(xError) * kP;
                commandedSpeed = Math.max(minSpeed, Math.min(maxSpeed, commandedSpeed));
                forward = Math.signum(xError) * commandedSpeed;
            }

            // Proportional strafe control with deadband and min/max speed clamp
            if (Math.abs(yError) > buffer) {
                double commandedSpeed = Math.abs(yError) * kP;
                commandedSpeed = Math.max(minSpeed, Math.min(maxSpeed, commandedSpeed));
                strafe = Math.signum(yError) * commandedSpeed;
            }

            // Stop once both axes are within tolerance
            if (forward == 0 && strafe == 0) {
                break;
            }

            drive.driveCommand(strafe, forward, 0);

            telemetry.addLine(String.format("Target:  (%.1f, %.1f)", targetX, targetY));
            telemetry.addLine(String.format("Current: (%.1f, %.1f)", currentX, currentY));
            telemetry.addLine(String.format("Error:   (%.1f, %.1f)", xError, yError));
            telemetry.addLine(String.format("Drive:   F %.2f | S %.2f", forward, strafe));
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
