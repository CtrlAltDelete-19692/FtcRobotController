package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

@Autonomous(name = "DecodeAutonomous")
public class DecodeAutonomous extends LinearOpMode {

    private Hardware hw;
    private int teamTagId = 20;

    private Drive drive;
    private Launcher launcher;
    private static final long LOADER_PULSE_MS = 350;  // feed one note
    private static final long LOADER_GAP_MS = 300;  // gap between shots
    private static final long QUARTER_TURN_MS = 800;
    private static final long FOOT_MS = 415;

    @Override
    public void runOpMode() throws InterruptedException {
        boolean isBlue = true;
        boolean squareStart = true;

        while (!isStarted()) {
            if (gamepad1.x) isBlue = true;
            if (gamepad1.b) isBlue = false;

            if (gamepad1.dpad_left) squareStart = true;
            if (gamepad1.dpad_right) squareStart = false;

            telemetry.addLine("Team Color");
            telemetry.addLine((isBlue ? "\uD83D\uDFE6 Blue" : "\uD83D\uDFE5 Red") + "     (X or B)");
            telemetry.addLine();
            telemetry.addLine("Start Position");
            telemetry.addLine((squareStart ? "Square" : "Goal") + "     (D-Pad Left or D-Pad Right)");
            telemetry.update();
        }

        hw = new Hardware();
        hw.setup(hardwareMap);

        int pipeline = 0; // Blue
        teamTagId = 20;
        if (! isBlue) {
            pipeline = 1; // Red
            teamTagId = 24;
        }
        hw.aprilTag.start();
        hw.limelight.pipelineSwitch(pipeline);

        drive = new Drive(hw);
        launcher = new Launcher(hw);

        waitForStart();

        if (opModeIsActive()) {
            if (squareStart) { // Start in Square
                driveForTime(0, 1, 0, FOOT_MS * 3);
                driveForTime(0, 0, 1, QUARTER_TURN_MS);
                //driveForTime(-1, 1, 1, QUARTER_TURN_MS);
                //driveForTime(0, 1, 0, 500);
            } else { // Start at Goal
                driveForTime(0, 1, 0, FOOT_MS * 4);
                driveForTime(0, 0, -1, QUARTER_TURN_MS * 2);
                centerOnTag(1500);
                spinUpLauncher((int)(Launcher.TICKS_PER_FOOT * 4), 5000);
                pulseLoader(LOADER_PULSE_MS, LOADER_GAP_MS);
                pulseLoader(LOADER_PULSE_MS, LOADER_GAP_MS);
            }

            stopAllMotors();
        }
    }

    private void driveForTime(double strafe, double forward, double rotate, double durationMs) {
        long start = System.currentTimeMillis();
        while (opModeIsActive() && System.currentTimeMillis() - start < durationMs) {
            drive.driveCommand(strafe, forward, rotate);
            idle();
        }

        driveStop();
    }

    private void driveStop() {
        drive.driveCommand(0, 0, 0);
    }

    private void spinUpLauncher(int backupTicksPerSecond, long spinupMs) {
        if (hw.launcher == null) return;

        launcher.autoLauncher(backupTicksPerSecond);

        long start = System.currentTimeMillis();
        while (opModeIsActive() && System.currentTimeMillis() - start < spinupMs) {
            telemetry.addLine(String.format("Launcher: %.0f", hw.launcher.getVelocity()));
            idle();
        }
    }

    private void pulseLoader(long pulseMs, long gapMs) {
        if (hw.loader == null) return;

        // Feed
        hw.loader.setPower(0.5); // same as your TeleOp
        sleep(pulseMs);

        // Stop
        hw.loader.setPower(0.0);
        sleep(gapMs);
    }

    private void centerOnTag(long durationMs) {
        long start = System.currentTimeMillis();
        while (opModeIsActive() && System.currentTimeMillis() - start < durationMs) {
            double rotate = drive.getAutoRotate(0);  // re-sample tx each loop
            drive.driveCommand(0, 0, rotate);
            idle();
        }

        driveStop();
    }

    private void stopAllMotors() {
        driveStop();

        if (hw.launcher != null) {
            hw.launcher.setVelocity(0);
        }
        if (hw.loader != null) {
            hw.loader.setPower(0);
        }
        if (hw.intake != null) {
            hw.intake.setPower(0);
        }
        if (hw.leftViperSlideMotor != null) {
            hw.leftViperSlideMotor.setPower(0);
        }
        if (hw.rightViperSlideMotor != null) {
            hw.rightViperSlideMotor.setPower(0);
        }
    }
}
