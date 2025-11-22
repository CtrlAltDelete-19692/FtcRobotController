package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

@Autonomous(name = "DecodeAutonomous")
public class DecodeAutonomous extends LinearOpMode {

    private Hardware hw;
    private int teamTagId = 20;

    private Drive drive;
    private Launcher launcher;
    private static final long LOADER_PULSE_MS = 700;  // feed one note
    private static final long LOADER_GAP_MS = 640;  // gap between shots
    private static final long FOOT_MS = 415;

    private boolean dpadPressed = false;

    @Override
    public void runOpMode() throws InterruptedException {
        boolean isBlue = true;
        boolean startAtBack = true;
        int pauseTimeSeconds = 9;

        while (!isStarted()) {
            if (gamepad1.x) isBlue = true;
            if (gamepad1.b) isBlue = false;

            if (gamepad1.dpad_left) startAtBack = true;
            if (gamepad1.dpad_right) startAtBack = false;

            dpadPressed = gamepad1.dpad_up || gamepad1.dpad_down;
            if (gamepad1.dpad_up) pauseTimeSeconds += 1;
            if (gamepad1.dpad_down) pauseTimeSeconds -= 1;
            if (pauseTimeSeconds >= 10) {
                pauseTimeSeconds = 10;
            }

            telemetry.addLine("Team Color");
            telemetry.addLine((isBlue ? "\uD83D\uDFE6 Blue" : "\uD83D\uDFE5 Red") + "     (X or B)");
            telemetry.addLine();
            telemetry.addLine("Start Position");
            telemetry.addLine((startAtBack ? "Back" : "Front") + "     (D-Pad Left or D-Pad Right)");
            telemetry.addLine();
            //telemetry.addLine("Pause Time");
            //telemetry.addLine(String.format("%d seconds", pauseTimeSeconds));
            telemetry.update();
        }

        hw = new Hardware();
        hw.setup(hardwareMap);

        int pipeline = 0; // Blue
        int direction = -1; // We program our moves based on Red, and multiple by direction (-1) to reverse rotation / strafing for blue
        teamTagId = 20;
        if (! isBlue) {
            pipeline = 1; // Red
            direction = 1;
            teamTagId = 24;
        }
        hw.aprilTag.start();
        hw.limelight.pipelineSwitch(pipeline);

        drive = new Drive(hw);
        launcher = new Launcher(hw);

        waitForStart();

        if (opModeIsActive()) {
            if (startAtBack) { // Start at back triangle
                // Position for shooting
                driveForward(200, direction);
                rotateRight(200, direction);
                centerOnTag(1000);

                // Spin up and shoot
                spinUpLauncher(10, 5000);
                pulseLoader();
                pulseLoader();
                pulseLoader();
                pulseLoader();
                pulseLoader();
                pulseLoader();
                stopLauncher();
                  //shoot(5, 5000);

                // Pause out of the way til the end of auto
                sleep(9000);

                // Get out of shooting triangle
                driveForward(FOOT_MS * 2, direction);
                rotateRight(570, direction);
            } else { // Start at Goal / large white triangle
                // Position for shooting
                driveBackward(FOOT_MS * 3.5, direction);
                strafeRight(500, direction);
                centerOnTag(1000);

                // Spin up and shoot
                spinUpLauncher(4, 4000);
                pulseLoader();
                pulseLoader();
                pulseLoader();
                pulseLoader();
                pulseLoader();
                pulseLoader();
                stopLauncher();

                // Pause out of the way til the end of auto
                driveForward(FOOT_MS * 3, direction);
                sleep(9000);

                // Get out of shooting triangle
                driveBackward(FOOT_MS * 3.5, direction);
                strafeRight(1100, direction);
                rotateRight(400, direction);
            }

            stopAllMotors();
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

    private void driveBackward(double durationMs, int direction) {
        driveForTime(0, -1, 0, durationMs, direction);
    }

    private void rotateLeft(double durationMs, int direction) {
        driveForTime(0, 0, -1, durationMs, direction);
    }

    private void rotateRight(double durationMs, int direction) {
        driveForTime(0, 0, 1, durationMs, direction);
    }

    private void strafeLeft(double durationMs, int direction) {
        driveForTime(-1, 0, 0, durationMs, direction);
    }

    private void strafeRight(double durationMs, int direction) {
        driveForTime(1, 0, 0, durationMs, direction);
    }

    private void sleep(double durationMs) {
        long start = System.currentTimeMillis();
        while (opModeIsActive() && System.currentTimeMillis() - start < durationMs) {
            idle();
        }

        driveStop();
    }

    private void driveStop() {
        drive.driveCommand(0, 0, 0);
    }

    private void spinUpLauncher(int defaultFeet, long spinupMs) {
        if (hw.launcher == null) return;

        launcher.autoLauncher((int)(defaultFeet * Launcher.TICKS_PER_FOOT));

        long start = System.currentTimeMillis();
        while (opModeIsActive() && System.currentTimeMillis() - start < spinupMs) {
            telemetry.addLine(String.format("Launcher: %.0f", hw.launcher.getVelocity()));
            telemetry.addLine(String.format("Target: %.0f", launcher.launcherVelocity));
            telemetry.update();
            idle();
        }
    }

    public void stopLauncher() {
        launcher.stopLauncher();
    }

    private void pulseLoader() {
        if (hw.loader == null) return;

        // Feed
        hw.loader.setPower(0.5); // same as your TeleOp
        sleep(LOADER_PULSE_MS);

        // Stop
        hw.loader.setPower(0.0);
        sleep(LOADER_GAP_MS);
    }

    private void shoot(int shots, int defaultFeet) {
        spinUpLauncher(defaultFeet, 5000);

        while (shots > 0) {
            if (hw.launcher.getVelocity() > launcher.launcherVelocity * 0.94) {
                hw.loader.setPower(0.5);
                shots = shots - 1;
                sleep(LOADER_GAP_MS);
            } else {
                hw.loader.setPower(0);
            }
            idle();
        }

        hw.loader.setPower(0);
        stopLauncher();
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
