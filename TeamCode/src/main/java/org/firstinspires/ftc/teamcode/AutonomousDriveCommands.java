package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

public class AutonomousDriveCommands {
    private final LinearOpMode opMode;
    private final Drive drive;

    public AutonomousDriveCommands(LinearOpMode opMode, Drive drive) {
        this.opMode = opMode;
        this.drive = drive;
    }

    // Mirroring changes strafe and turn; forward/backward are the same for both alliances.
    public void driveForTime(double strafe, double forward, double rotate,
                             double durationMs, int allianceDirection) {
        long start = System.currentTimeMillis();
        while (opMode.opModeIsActive() && System.currentTimeMillis() - start < durationMs) {
            drive.driveCommand(strafe * allianceDirection, forward, rotate * allianceDirection);
            opMode.idle();
        }
        stop();
    }

    public void forward(double durationMs, int allianceDirection) {
        driveForTime(0, 1, 0, durationMs, allianceDirection);
    }

    public void backward(double durationMs, int allianceDirection) {
        driveForTime(0, -1, 0, durationMs, allianceDirection);
    }

    public void strafeLeft(double durationMs, int allianceDirection) {
        driveForTime(-1, 0, 0, durationMs, allianceDirection);
    }

    public void strafeRight(double durationMs, int allianceDirection) {
        driveForTime(1, 0, 0, durationMs, allianceDirection);
    }

    public void turnLeft(double durationMs, int allianceDirection) {
        driveForTime(0, 0, -1, durationMs, allianceDirection);
    }

    public void turnRight(double durationMs, int allianceDirection) {
        driveForTime(0, 0, 1, durationMs, allianceDirection);
    }

    public boolean turnDegrees(double degrees) {
        double startHeading = -drive.imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES);
        double targetHeading = startHeading + degrees;
        long startTime = System.currentTimeMillis();
        long lastReportTime = 0;
        long timeoutMs = Math.abs(degrees) > 135 ? 5000 : 3500;

        while (opMode.opModeIsActive() && System.currentTimeMillis() - startTime < timeoutMs) {
            double rawYaw = drive.imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES);
            double heading = -rawYaw;
            double error = headingError(targetHeading, heading);
            if (Math.abs(error) <= 3) {
                stop();
                showTurnTelemetry("Aligned", rawYaw, targetHeading, error);
                return true;
            }
            double power = Math.min(0.35, Math.max(0.12, Math.abs(error) * 0.01));
            drive.driveCommand(0, 0, Math.copySign(power, error));
            if (System.currentTimeMillis() - lastReportTime >= 100) {
                showTurnTelemetry("Turning", rawYaw, targetHeading, error);
                lastReportTime = System.currentTimeMillis();
            }
            opMode.idle();
        }
        stop();
        if (opMode.opModeIsActive()) {
            double rawYaw = drive.imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES);
            showTurnTelemetry("Timed out", rawYaw, targetHeading, headingError(targetHeading, -rawYaw));
        }
        return false;
    }

    private double headingError(double targetHeading, double heading) {
        double error = targetHeading - heading;
        while (error > 180) error -= 360;
        while (error < -180) error += 360;
        return error;
    }

    private void showTurnTelemetry(String state, double rawYaw, double targetHeading, double error) {
        opMode.telemetry.addData("Turn", state);
        opMode.telemetry.addData("IMU yaw / used heading / target", "%.1f° / %.1f° / %.1f°",
                rawYaw, -rawYaw, targetHeading);
        opMode.telemetry.addData("Error", "%+.1f°", error);
        opMode.telemetry.update();
    }

    public void stop() {
        drive.driveCommand(0, 0, 0);
    }
}
