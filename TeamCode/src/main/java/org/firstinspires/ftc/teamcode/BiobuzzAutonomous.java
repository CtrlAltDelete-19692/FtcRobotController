package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.hardware.DcMotor;

@Autonomous(name = "BiobuzzAutonomous")
public class BiobuzzAutonomous extends CtrlAltDelOpMode {
    private static final long SHOOT_READY_TIMEOUT_MS = 4000;
    private static final long FEED_MS_PER_BALL = 1000;
    private static final long FORWARD_MS_PER_FOOT = 440; // Starting estimate from Decode; tune on Biobuzz.
    private static final long STRAFE_MS_PER_FOOT = 500;  // Starting estimate from Decode; tune on Biobuzz.
    private static final long FLOWER_PULLER_MOVE_MS = 800;
    private static final long FLOWER_COLLECTION_PAUSE_MS = 3000;
    private static final long BALL_PURGE_MS = 1500;
    private static final double COLLECTION_FORWARD_FEET = 1.4;
    private static final double COLLECTION_FORWARD_SPEED_SCALE = 0.5; // Fraction of normal autonomous power.
    private static final double COLLECTION_FORWARD_TIME_TRIM = 1.0; // Increase if short; decrease if overshooting.
    private static final double COLLECTION_SHAKE_INCHES = 4.0; // Travel in each direction; returns to collection position.
    private static final int COLLECTION_SHAKE_CYCLES = 3; // Set to 0 to disable.

    private Drive drive;
    private AutonomousDriveCommands autoDrive;
    private Intake intake;
    private FlowerPuller flowerPuller;
    private RotatingLauncher launcher;
    private BallPurge ballPurge;
    private Limelight3A limelight;
    private BiobuzzTeamSelection team;
    private int ballCount = 8;
    private boolean shootFirst = true;
    private boolean lastCountLeft;
    private boolean lastCountRight;
    private boolean shootingActive;
    private boolean collectionActive;
    private double fixedFeedVelocity = Double.NaN;
    private enum WaitMode { ELAPSED, FEED_CYCLE }
    private enum ShotStart { WAITING, READY, TIMED_OUT }
    private ShotStart shotStart = ShotStart.WAITING;

    @Override
    public void runOpMode() {
        killMotors = false;
        drive = new Drive(hardwareMap, Drive.RobotProfile.BIOBUZZ, false);
        autoDrive = new AutonomousDriveCommands(this, drive);
        intake = new Intake(hardwareMap, DcMotor.Direction.FORWARD);
        intake.disableJamDetection();
        flowerPuller = new FlowerPuller(hardwareMap);
        launcher = new RotatingLauncher(hardwareMap, "Pollen");
        launcher.setFeedIntake(intake);
        ballPurge = new BallPurge(intake, launcher);
        limelight = hardwareMap.tryGet(Limelight3A.class, "Limelight");
        if (limelight != null) {
            limelight.setPollRateHz(100);
            limelight.start();
        }
        team = new BiobuzzTeamSelection(limelight, telemetry);
        team.select(true);
        autoDrive.setPeriodicUpdate(this::updateNonShootingAim);

        TelemetryDashboard dashboard = new TelemetryDashboard(telemetry);
        while (!isStarted() && !isStopRequested()) {
            if (gamepad1.x) team.select(true);
            if (gamepad1.b) team.select(false);
            if (gamepad1.dpad_up) shootFirst = true;
            if (gamepad1.dpad_down) shootFirst = false;
            if (gamepad1.dpad_left && !lastCountLeft) ballCount--;
            if (gamepad1.dpad_right && !lastCountRight) ballCount++;
            lastCountLeft = gamepad1.dpad_left;
            lastCountRight = gamepad1.dpad_right;
            if (ballCount < 0) {
                ballCount = 0;
            } else if (ballCount > 8) {
                ballCount = 8;
            }

            dashboard.addStatus(team.isBlue(), drive.getDriveMode());
            telemetry.addData("Program", (shootFirst ? "Shoot First" : "Shoot Second")
                    + "          (d-pad up / down)");
            telemetry.addData("Balls", ballCount + "          (d-pad left / right)");
            telemetry.addLine();
            telemetry.addLine();
            telemetry.addLine();
            telemetry.addLine();
            telemetry.addLine();
            telemetry.addData("Hopper", intake.hasHopper() ? "✅ Configured" : "⚠ Missing");
            telemetry.addData("Loader", launcher.hasLoader() ? "✅ Configured" : "⚠ Missing");
            telemetry.addData("FlowerPuller", flowerPuller.isConfigured() ? "✅ Configured" : "⚠ Missing");
            dashboard.update();
        }

        if (isStopRequested()) return;
        waitForStart();
        flowerPuller.setUp();
        telemetry.clearAll();

        try {
            drive.imu.resetYaw();
            autoDrive.stop();
            if (!intake.hasHopper()) {
                telemetry.addLine("Cannot collect: Hopper is not configured.");
                telemetry.update();
                return;
            }
            if (!shootInitialBalls()) return;
            if (!flowerPuller.isConfigured()) {
                telemetry.addLine("FlowerPuller missing; collection skipped.");
                telemetry.update();
                return;
            }
            if (!approachFlower() || !collectFromFlower() || !finishRoute()) return;

            drive.imu.resetYaw();

            telemetry.addLine("Autonomous route complete");
            telemetry.update();
        } finally {
            ballPurge.setRunning(false);
            collectionActive = false;
            launcher.setLoaderRunning(false);
            intake.setCollectionRunning(false);
            launcher.stopLauncher();
            launcher.stopTurret();
            flowerPuller.setUp();
            autoDrive.stop();
        }
    }

    private boolean shootInitialBalls() {
        if (ballCount == 0) {
            telemetry.addLine("0 balls selected; continuing to the flower.");
            telemetry.update();
            return true;
        }
        if (!launcher.hasLoader()) {
            telemetry.addLine("Cannot shoot: Loader is not configured.");
            telemetry.update();
            return false;
        }
        if (!shootBalls(ballCount)) return false;
        ballPurge.setRunning(true);
        try {
            return waitForElapsed(BALL_PURGE_MS, "Clearing remaining balls");
        } finally {
            ballPurge.setRunning(false);
        }
    }

    private boolean approachFlower() {
        return shootFirst ? strafeRight(2.85) && forward(0.35)
                : strafeRight(1.5) && turn(-90) && strafeRight(1.3);
    }

    private boolean collectFromFlower() {
        flowerPuller.setDown();
        if (!waitForElapsed(FLOWER_PULLER_MOVE_MS, "Lowering FlowerPuller")) return false;
        collectionActive = true;
        intake.setCollectionRunning(true);
        if (!collectionForward()) return false;
        if (!waitForElapsed(FLOWER_COLLECTION_PAUSE_MS, "Collecting pollen")) return false;
        if (!backward(0.5)) return false;
        flowerPuller.setUp();
        return waitForElapsed(FLOWER_PULLER_MOVE_MS, "Raising FlowerPuller");
    }

    private boolean finishRoute() {
        if (shootFirst) {
            return strafeRight(3.8) && turn(180) && backward(0.8);
        }
        return turn(90) && backward(2.4);
    }

    private boolean shootBalls(int count) {
        fixedFeedVelocity = Double.NaN;
        shotStart = ShotStart.WAITING;
        launcher.startAutonomousAiming();
        shootingActive = true;
        try {
            if (!waitForShotReadyOrTimeout()) return false;

            fixedFeedVelocity = launcher.launcherVelocity;
            intake.setHopperAfterReverseForwardEnabled(true);
            launcher.setLoaderRunning(true);
            long feedDurationMs = Intake.HOPPER_LOADER_FEED_CYCLE_ENABLED
                    ? Intake.HOPPER_LOADER_CYCLE_MS * count // Let the final cycle finish without staging another ball.
                    : FEED_MS_PER_BALL * count;
            if (!waitForFeedCycles(feedDurationMs, "Feeding " + count + " balls")) return false;
            return true;
        } finally {
            shootingActive = false;
            fixedFeedVelocity = Double.NaN;
            launcher.setLoaderRunning(false);
            intake.setCollectionRunning(false);
            intake.setHopperAfterReverseForwardEnabled(true);
            launcher.stopTurret();
            launcher.stopLauncher();
        }
    }

    private boolean forward(double feet) {
        autoDrive.forward(feet * FORWARD_MS_PER_FOOT, 1);
        return opModeIsActive();
    }

    private boolean collectionForward() {
        // Half power starts with twice the duration; trim compensates for real motor/traction behavior.
        double durationMs = COLLECTION_FORWARD_FEET * FORWARD_MS_PER_FOOT
                / COLLECTION_FORWARD_SPEED_SCALE * COLLECTION_FORWARD_TIME_TRIM;
        if (!collectionDrive(1, durationMs)) return false;
        if (!backward(0.3)) return false;
        double shakeDurationMs = COLLECTION_SHAKE_INCHES / 12.0 * FORWARD_MS_PER_FOOT
                / COLLECTION_FORWARD_SPEED_SCALE * COLLECTION_FORWARD_TIME_TRIM;
        for (int cycle = 0; cycle < COLLECTION_SHAKE_CYCLES; cycle++) {
            if (!collectionDrive(1, shakeDurationMs) || !collectionDrive(-1, shakeDurationMs)) return false;
        }
        return backward(0.2);
    }

    private boolean collectionDrive(double direction, double durationMs) {
        return collectionMove(0, direction, durationMs, COLLECTION_FORWARD_SPEED_SCALE);
    }

    private boolean collectionMove(double strafe, double forward, double durationMs, double speedScale) {
        long start = System.currentTimeMillis();
        try {
            while (opModeIsActive() && !killMotors && System.currentTimeMillis() - start < durationMs) {
                updateNonShootingAim();
                intake.setCollectionRunning(true);
                intake.updateHopperStall();
                drive.driveCommand(strafe, forward, 0, speedScale);
                idle();
            }
        } finally {
            autoDrive.stop();
        }
        return opModeIsActive() && !killMotors;
    }

    private boolean backward(double feet) {
        if (collectionActive) return collectionMove(0, -1, feet * FORWARD_MS_PER_FOOT, 1.0);
        autoDrive.backward(feet * FORWARD_MS_PER_FOOT, 1);
        return opModeIsActive();
    }

    private boolean strafeRight(double feet) {
        autoDrive.strafeRight(feet * STRAFE_MS_PER_FOOT, 1);
        return opModeIsActive();
    }

    private boolean turn(double degrees) {
        return autoDrive.turnDegrees(degrees);
    }

    private boolean waitForShotReadyOrTimeout() {
        long start = System.currentTimeMillis();
        while (opModeIsActive()
                && System.currentTimeMillis() - start < SHOOT_READY_TIMEOUT_MS) {
            updateShooting();
            if (launcher.readyToLaunch) {
                shotStart = ShotStart.READY;
                telemetry.addData("Shot start", "Ready");
                telemetry.update();
                return true;
            }
            telemetry.addData("Shot start", "Waiting for ready");
            launcher.addCompetitionTelemetry(telemetry);
            telemetry.update();
            idle();
        }
        if (opModeIsActive()) {
            shotStart = ShotStart.TIMED_OUT;
            telemetry.addData("Shot start", "Timed out after %d ms; shooting at current speed",
                    SHOOT_READY_TIMEOUT_MS);
            telemetry.update();
        }
        return opModeIsActive();
    }

    private void updateShooting() {
        launcher.updateAutonomousShooting(limelight == null ? null : limelight.getLatestResult(),
                team.isBlue(), fixedFeedVelocity);
    }

    private void updateNonShootingAim() {
        if (!shootingActive) {
            launcher.updateVisibleTagTracking(limelight == null ? null : limelight.getLatestResult(), team.isBlue());
        }
    }

    private boolean waitForElapsed(long durationMs, String phase) {
        return waitForDuration(durationMs, phase, WaitMode.ELAPSED);
    }

    private boolean waitForFeedCycles(long durationMs, String phase) {
        return waitForDuration(durationMs, phase, WaitMode.FEED_CYCLE);
    }

    private boolean waitForDuration(long durationMs, String phase, WaitMode mode) {
        boolean pulseHopper = mode == WaitMode.FEED_CYCLE;
        long start = System.currentTimeMillis();
        while (opModeIsActive() && (pulseHopper ? intake.getLoaderFeedElapsedMs()
                : System.currentTimeMillis() - start) < durationMs) {
            if (pulseHopper) intake.setHopperAfterReverseForwardEnabled(
                    intake.getLoaderFeedElapsedMs() < durationMs - Intake.HOPPER_LOADER_CYCLE_MS);
            if (shootingActive) {
                updateShooting();
                launcher.setLoaderRunning(true);
            } else {
                updateNonShootingAim();
            }
            if (collectionActive && !killMotors) intake.setCollectionRunning(true);
            intake.updateHopperStall();
            telemetry.addData("Autonomous", phase);
            if (intake.isHopperStalled()) telemetry.addLine("Hopper stalled; feed stopped");
            if (shootingActive) {
                telemetry.addData("Shot start", shotStart == ShotStart.READY ? "Ready" : "Timed out");
                launcher.addCompetitionTelemetry(telemetry);
                if (launcher.isUsingNoTagFallback()) telemetry.addData("No tag", "Using %.1f ft shooting speed",
                        RotatingLauncher.NO_TAG_DISTANCE_FEET);
            } else {
                telemetry.addData("Flywheel", "%.0f / %.0f ticks/s",
                        launcher.launcher.getVelocity(), launcher.launcherVelocity);
            }
            telemetry.update();
            idle();
        }
        return opModeIsActive();
    }

}
