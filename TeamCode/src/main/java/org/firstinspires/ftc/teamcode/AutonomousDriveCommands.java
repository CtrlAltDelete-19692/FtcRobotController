package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

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

    public void stop() {
        drive.driveCommand(0, 0, 0);
    }
}
