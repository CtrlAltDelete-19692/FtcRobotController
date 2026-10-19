package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;

@Autonomous(name = "DecodeAutonomous")
public class DecodeAutonomous extends CtrlAltDelOpMode {
    private int teamTagId = 20;

    private Drive drive;
    private Launcher launcher;
    private Intake intake;

    public GoBildaPinpointDriver pinpoint;

    private static final String[] AUTO_PROGRAMS = { "Back", "Front" };
    private int autoProgramIndex = 1;

    private static final long LOADER_GAP_MS = 800;  // gap between shots
    private static final long FOOT_MS = 350; // Desmond's living room carpet
    //private static final long FOOT_MS = 415;
    private static final long STRAFE_FOOT_MS = 500; // Desmond's living room carpet
    //private static final long STRAFE_FOOT_MS = 600;

    private boolean dpadPressedLast = false;
    private boolean dpadLRPressedLast = false;

    boolean lastLaunch = false;
    boolean runningSequence = false;

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

        int pipeline = 0; // Blue
        int direction = -1; // We program our moves based on Red, and multiple by direction (-1) to reverse rotation / strafing for blue
        teamTagId = 20;
        if (! isBlue) {
            pipeline = 1; // Red
            direction = 1;
            teamTagId = 24;
        }

        drive = new Drive(hardwareMap, "", true);
        AprilTag aprilTag = new AprilTag(hardwareMap);
        aprilTag.pipelineSwitch(pipeline);
        launcher = new Launcher(hardwareMap, aprilTag, telemetry);
        intake = new Intake(hardwareMap, DcMotor.Direction.FORWARD);

        pinpoint = hardwareMap.get(GoBildaPinpointDriver.class, "Odo");

        // 1. Initialize settings (Check user guide for your specific pod's ticks/mm)
        //pinpoint.setXOffset(-120.0); // Lateral pod offset
        //pinpoint.setYOffset(-120.0); // Forward pod offset
        pinpoint.setEncoderResolution(GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_4_BAR_POD);
        //pinpoint.setEncoderDirections(GoBildaPinpointDriver.EncoderDirection.FORWARD, GoBildaPinpointDriver.EncoderDirection.FORWARD);

        waitForStart();

        telemetry.update(); // Clear previous telemetry data (selection screen)

        String selectedProgram = AUTO_PROGRAMS[autoProgramIndex];

        if (opModeIsActive()) {
            pinpoint.resetPosAndIMU(); // Sets current position to (0,0,0)
            pinpoint.update();

            if (selectedProgram.equals("Back")) {
                //backOdo();

                back(direction);
            } else if (selectedProgram.equals("Front")) { // Start at Goal / large white triangle
                //frontOdo();

                front(direction);
            }

            stopAllMotors();
        }
    }

    private void front(int direction) {
        // Position for shooting
        driveBackward(FOOT_MS * 3.5, direction);
        rotateLeft(300, direction);

        spinUpLauncher(4, 4000);
        intake.spin(true);

        centerOnTag(1000);
        pulseLoader(5);

        // Collect Artifact Batch 1
        turnToHeading(0);
        //rotateRight(400, direction);

        strafeRight(STRAFE_FOOT_MS * 2.2, direction);
        driveForward(FOOT_MS * 3.2, direction); // Collect Artifacts
        driveBackward(FOOT_MS * 3.2, direction);
        strafeLeft(STRAFE_FOOT_MS * 2.2, direction);

        rotateLeft(300, direction);

        centerOnTag(1000);
        pulseLoader(5);

        // Collect Artifact Batch 2
        turnToHeading(0);
        //rotateRight(400, direction);

        strafeRight(STRAFE_FOOT_MS * (2.2 + 2), direction);
        driveForward(FOOT_MS * 3.2, direction); // Collect Artifacts
        driveBackward(FOOT_MS * 3.2, direction);
        strafeLeft(STRAFE_FOOT_MS * (2.2 + 2), direction);

        turnToHeading(0);

        stopLauncher();
        intake.spin(false);
    }

    private void back(int direction) {
        // Position for shooting
        driveForTime(-1, 1, -1, FOOT_MS, direction);
        //strafeLeft(STRAFE_FOOT_MS * 0.5, direction);
        //driveForward(FOOT_MS * 0.5);
        //rotateLeft(400, direction);

        spinUpLauncher(10, 5000);
        intake.spin(true);

        centerOnTag(1000);
        pulseLoader(5);

        // Collect Artifact Batch 1
        turnToHeading(0);

        strafeLeft(STRAFE_FOOT_MS * 1.7, direction);
        driveForward(FOOT_MS * 4, direction); // Collect Artifacts
        driveBackward(FOOT_MS * 4, direction);
        strafeRight(STRAFE_FOOT_MS * 1.7, direction);

        rotateLeft(400, direction);

        centerOnTag(1000);
        pulseLoader(5);

        // Collect Artifact Batch 2
        turnToHeading(0);

        strafeLeft(STRAFE_FOOT_MS * (1.7 + 2), direction);
        driveForward(FOOT_MS * 4, direction); // Collect Artifacts
        driveBackward(FOOT_MS * 4, direction);
        strafeRight(STRAFE_FOOT_MS * (1.7 + 2), direction);

        rotateLeft(400, direction);

        centerOnTag(1000);
        pulseLoader(5);

        // Get out of shooting triangle
        turnToHeading(0);
        strafeLeft(STRAFE_FOOT_MS * 1.5, direction);
        driveForward(FOOT_MS * 1.5, direction);

        stopLauncher();
        intake.spin(false);
    }

    private void frontOdo() {
        takeCloseShots(4);
//
//                goToPose(24, -30);
//                intake.spin(true);
//                goToPose(24, 0);
//                intake.spin(false);
//
//                takeCloseShots(4);

//                goToPose(48, -30);
//                intake.spin(true);
//                goToPose(48, 0);
//                intake.spin(false);
//
//                takeCloseShots(4);
    }

    private void backOdo() {
        takeFarShots(4);

        goToPose(-24, 0);
        intake.spin(true);
        goToPose(-24, 36);
        intake.spin(false);

        takeFarShots(4);
    }

    private void takeCloseShots(int shotCount) {
        goToPose(10, -30);
        turnToHeading(-40);
        spinUpLauncher(3, 5000);
        intake.spin(true);
        pulseLoader(shotCount);
        intake.spin(false);
        turnToHeading(0);
        stopLauncher();
    }
    private void takeFarShots(int shotCount) {
        turnToHeading(-30);
        spinUpLauncher(3, 5000);
        intake.spin(true);
        pulseLoader(shotCount);
        intake.spin(false);
        turnToHeading(0);
        stopLauncher();
    }

    private void goToPose(double targetY, double targetX) {
        final double buffer = 2.0; // inches
        final double minSpeed = 0.1;
        final double maxSpeed = 0.2;
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

            boolean xFlip = false;
            boolean yFlip = true;
            double currentX = pos.getX(DistanceUnit.INCH) * (xFlip ? -1 : 1);
            double currentY = pos.getY(DistanceUnit.INCH) * (yFlip ? -1 : 1);

            // Error from target
            double xError = targetX - currentX;
            double yError = targetY - currentY;

            double strafe = 0;
            double forward = 0;

            int xDirection = xError > 0 ? 1 : -1;
            if (Math.abs(xError) > buffer) {
                forward = Math.abs(xError) * kP;

                if (forward < minSpeed) {
                    forward = minSpeed;
                }
                if (strafe > maxSpeed) {
                    forward = maxSpeed;
                }

                forward *= xDirection;
            } else {
                forward = 0;
            }

            int yDirection = yError > 0 ? 1 : -1;
            if (Math.abs(yError) > buffer) {
                strafe = Math.abs(yError) * kP;

                if (strafe < minSpeed) {
                    strafe = minSpeed;
                }
                if (strafe > maxSpeed) {
                    strafe = maxSpeed;
                }

                strafe *= yDirection;
            } else {
                strafe = 0;
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

            //double currentHeading = pos.getHeading(AngleUnit.DEGREES) * -1; // Flip
            double currentHeading = -drive.imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES);

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
        if (launcher.launcher == null) return;

        launcher.autoLauncher((int)(defaultFeet * Launcher.TICKS_PER_FOOT));

        long start = System.currentTimeMillis();
        while (opModeIsActive()
                && System.currentTimeMillis() - start < spinupMs
                && !launcher.upToSpeed()) {
            telemetry.addLine(String.format("Launcher: %.0f", launcher.launcher.getVelocity()));
            telemetry.addLine(String.format("Target: %.0f", launcher.launcherVelocity));
            telemetry.update();
            idle();
        }
    }

    public void stopLauncher() {
        launcher.stopLauncher();
    }

    private void pulseLoader(int shotCount) {

        if (launcher.loader == null) return;

        for (int i = 0; i < shotCount && opModeIsActive(); i++) {

            // Start launch sequence
            launcher.triggerLoader();

            // Wait for loader sequence to complete
            while (launcher.runningSequence && opModeIsActive()) {
                launcher.updateLoader();
                idle(); // Let FTC runtime breathe
            }

            // Wait for next ball to settle into launcher
            sleep(LOADER_GAP_MS);
        }
    }

    private void heartbeatShoot(){ //
        if (launcher.loader == null) return;

        int feedMs = 400;

        // Feed
        launcher.loader.setPower(0.3); // same as your TeleOp
        sleep(feedMs);

        launcher.loader.setPower(0.0); //Sleep
        sleep(feedMs);

        // Go Back
        launcher.loader.setPower(-0.3);
        sleep(feedMs);

        launcher.loader.setPower(0.0); //Sleep again to stop motor
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
                launcher.loader.setPower(0.5);
                shotInProgress = true;
                phaseStartTime = now;
                telemetry.addLine("First");
            } else if (shotInProgress) {
                boolean velocityDropped = !launcher.upToSpeed();
                boolean pulseTimedOut = now - phaseStartTime >= 3500; // 1500 is maximum pulse time

                if (velocityDropped || pulseTimedOut) {
                    launcher.loader.setPower(0);
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

        launcher.loader.setPower(0);
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

        if (launcher.launcher != null) {
            launcher.launcher.setVelocity(0);
        }
        if (launcher.loader != null) {
            launcher.loader.setPower(0);
        }
        if (intake.pickupMotor != null) {
            intake.pickupMotor.setPower(0);
        }
    }
}
