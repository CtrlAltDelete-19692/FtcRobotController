package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

@Autonomous(name = "DecodeAutonomous")
public class DecodeAutonomous extends LinearOpMode {

    private Hardware hw;
    private int teamTagId = 20;

    private Drive drive;
    private Launcher launcher;

    private static final String[] AUTO_PROGRAMS = { "Back", "Front" };
    private int autoProgramIndex = 1;

    private static final long LOADER_PULSE_MS = 700;  // feed one note
    private static final long LOADER_GAP_MS = 640;  // gap between shots
    private static final long FOOT_MS = 415;

    private boolean dpadPressedLast = false;
    private boolean dpadLRPressedLast = false;

    @Override
    public void runOpMode() throws InterruptedException {
        boolean isBlue = true;
        boolean startAtBack = true;
        int pauseTimeSeconds = 9;

        while (!isStarted()) {
            // Team color selection
            if (gamepad1.x) isBlue = true;
            if (gamepad1.b) isBlue = false;

            // Auto program selection (D-pad left/right)
//            boolean dpadLRPressed = gamepad1.dpad_left || gamepad1.dpad_right;
//            if (dpadLRPressed && !dpadLRPressedLast) {
//                if (gamepad1.dpad_right) {
//                    autoProgramIndex++;
//                    if (autoProgramIndex >= AUTO_PROGRAMS.length) {
//                        autoProgramIndex = 0;
//                    }
//                } else if (gamepad1.dpad_left) {
//                    autoProgramIndex--;
//                    if (autoProgramIndex < 0) {
//                        autoProgramIndex = AUTO_PROGRAMS.length - 1;
//                    }
//                }
//            }
//            dpadLRPressedLast = dpadLRPressed;

            if (gamepad1.dpad_left) {
                autoProgramIndex = 0; // Back
            } else if (gamepad1.dpad_right) {
                autoProgramIndex = 1; // Front
            }

            boolean dpadPressed = gamepad1.dpad_up || gamepad1.dpad_down;
            if (dpadPressed && !dpadPressedLast) {
                if (gamepad1.dpad_up) {
                    pauseTimeSeconds += 1;
                }
                if (gamepad1.dpad_down) {
                    pauseTimeSeconds -= 1;
                }
            }
            dpadPressedLast = dpadPressed;
            if (pauseTimeSeconds >= 10) {
                pauseTimeSeconds = 10;
            } else if (pauseTimeSeconds < 0) {
                pauseTimeSeconds = 0;
            }

            telemetry.addLine("Team Color");
            telemetry.addLine((isBlue ? "\uD83D\uDFE6 Blue" : "\uD83D\uDFE5 Red") + "     (X or B)");
            telemetry.addLine();

            telemetry.addLine("Auto Program");
            telemetry.addLine(AUTO_PROGRAMS[autoProgramIndex] + "     (D-Pad Left / Right)");
            telemetry.addLine();

            telemetry.addLine("Pause Time     (D-Pad Up / D-Pad Down)");
            telemetry.addLine(String.format("%d seconds", pauseTimeSeconds));
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

        telemetry.update(); // Clear previous telemetry data (selection screen)

        String selectedProgram = AUTO_PROGRAMS[autoProgramIndex];

        if (opModeIsActive()) {
            if (selectedProgram.equals("Back") || selectedProgram.equals("Back: New Launcher")) {
                // Position for shooting
                driveForward(200, direction);
                rotateRight(200, direction);
                centerOnTag(1000);

                // Spin up and shoot
                if (selectedProgram.equals("Back: New Launcher")) {
                    shoot(9, 4);
                } else {
                    // Original behavior: spin up + pulses
                    spinUpLauncher(10, 5000);
                    pulseLoader();
                    pulseLoader();
                    pulseLoader();
                    pulseLoader();
                    pulseLoader();
                    pulseLoader();
                    stopLauncher();
                }

                // Pause out of the way til the end of auto
                pause(pauseTimeSeconds * 1000);

                // Get out of shooting triangle
                driveForward(FOOT_MS * 2, direction);
                rotateRight(570, direction);
            } else if (selectedProgram.equals("Front") || selectedProgram.equals("Front: New Loader")) { // Start at Goal / large white triangle
                // Position for shooting
                driveBackward(FOOT_MS * 3.5, direction);
                strafeRight(500, direction);
                centerOnTag(1000);

                // Spin up and shoot
                if (selectedProgram.equals("Front: New Loader")) {
                    // Spin up and shoot
                    spinUpLauncher(4, 4000);
                    heartbeatShoot();
                    sleep(2000);
                    heartbeatShoot();
                    sleep(2000);
                    heartbeatShoot();
                    stopLauncher();
                } else {
                    spinUpLauncher(4, 4000);
                    pulseLoader();
                    pulseLoader();
                    pulseLoader();
                    pulseLoader();
                    pulseLoader();
                    pulseLoader();
                    stopLauncher();
                }

                // Pause out of the way til the end of auto
                driveForward(FOOT_MS * 3, direction);
                pause(pauseTimeSeconds * 1000);

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

    private void pause(long durationMs) {
        long start = System.currentTimeMillis();

        while (opModeIsActive()) {
            long elapsedMs   = System.currentTimeMillis() - start;
            long remainingMs = durationMs - elapsedMs;

            if (remainingMs <= 0) {
                break;
            }

            double remainingSec = remainingMs / 1000.0;

            telemetry.addData("Pause", "%.1f seconds left", remainingSec);
            telemetry.update();

            idle();
        }
    }

    private void driveStop() {
        drive.driveCommand(0, 0, 0);
    }

    private void spinUpLauncher(int defaultFeet, long spinupMs) {
        if (hw.launcher == null) return;

        launcher.autoLauncher((int)(defaultFeet * Launcher.TICKS_PER_FOOT));

        long start = System.currentTimeMillis();
        while (opModeIsActive()
                && System.currentTimeMillis() - start < spinupMs
                && !launcher.upToSpeed()) {
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

    private void heartbeatShoot(){ //
        if (hw.loader == null) return;

        int feedMs = 400;

        // Feed
        hw.loader.setPower(0.3); // same as your TeleOp
        sleep(feedMs);

        hw.loader.setPower(0.0); //Sleep
        sleep(feedMs);

        // Go Back
        hw.loader.setPower(-0.3);
        sleep(feedMs);

        hw.loader.setPower(0.0); //Sleep again to stop motor
    }

    private void shoot(int shots, int defaultFeet) {
        spinUpLauncher(defaultFeet, 5000);

        boolean shotInProgress = false;
        boolean inGap = false;
        long phaseStartTime = 0;

        while (shots > 0 && opModeIsActive()) {

            telemetry.addData("Shots", shots);
            telemetry.addData("upToSpeed", launcher.upToSpeed());
            telemetry.addData("shotInProgress", shotInProgress);
            telemetry.addData("inGap", inGap);

            long now = System.currentTimeMillis();

            if (!shotInProgress && !inGap && launcher.upToSpeed()) {
                hw.loader.setPower(0.5);
                shotInProgress = true;
                phaseStartTime = now;
                telemetry.addLine("First");
            } else if (shotInProgress) {
                boolean velocityDropped = !launcher.upToSpeed();
                boolean pulseTimedOut = now - phaseStartTime >= 3500; // 1500 is maximum pulse time

                if (velocityDropped || pulseTimedOut) {
                    hw.loader.setPower(0);
                    shotInProgress = false;
                    inGap = true;
                    phaseStartTime = now;
                }
                telemetry.addLine("Second");
            } else if (inGap) {
                if (now - phaseStartTime >= 3000) { // Small pause
                    inGap = false;
                    shots = shots - 1;
                }
                telemetry.addLine("Third");
            }
            telemetry.update();
            idle();
        }

        telemetry.addData("Im out!", shots);
        telemetry.update();

        hw.loader.setPower(0);
        stopLauncher();
    }

    private void centerOnTag(long durationMs) {
        long start = System.currentTimeMillis();
        while (opModeIsActive() && System.currentTimeMillis() - start < durationMs) {
            double rotate = drive.getAutoRotate(0, teamTagId);  // re-sample tx each loop
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
