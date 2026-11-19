package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.hardware.DcMotor;

@Autonomous(name = "BiobuzzAutonomous")
public class BiobuzzAutonomous extends CtrlAltDelOpMode {
    private static final long SHOOT_READY_TIMEOUT_MS = 5000;
    private static final long FEED_TARGET_LOSS_GRACE_MS = 250;
    private static final long FEED_SPEED_DROP_GRACE_MS = 150;
    private static final double FEED_MIN_SPEED_FRACTION = 0.80;
    private static final double FEED_RESUME_SPEED_FRACTION = 0.94;
    private static final long FEED_MS_PER_BALL = 1000;
    private static final long FORWARD_MS_PER_FOOT = 440; // Starting estimate from Decode; tune on Biobuzz.
    private static final long STRAFE_MS_PER_FOOT = 500;  // Starting estimate from Decode; tune on Biobuzz.
    private static final long FLOWER_PULLER_MOVE_MS = 800;
    private static final long FLOWER_COLLECTION_PAUSE_MS = 3000;
    private static final double COLLECTION_FORWARD_FEET = 1.4;
    private static final double COLLECTION_FORWARD_SPEED_SCALE = 0.5; // Fraction of normal autonomous power.
    private static final double COLLECTION_FORWARD_TIME_TRIM = 1.0; // Increase if short; decrease if overshooting.
    private static final double COLLECTION_SHAKE_INCHES = 4.0; // Travel in each direction; returns to collection position.
    private static final int COLLECTION_SHAKE_CYCLES = 3; // Set to 0 to disable.
    private static final double OPTIMISTIC_CROSS_FIELD_STRAFE_FEET = 18.0;

    private enum TipConfidence { OPTIMISTIC, PESSIMISTIC }
    private TipConfidence tipConfidence = TipConfidence.PESSIMISTIC;

    private Drive drive;
    private AutonomousDriveCommands autoDrive;
    private Intake intake;
    private FlowerPuller flowerPuller;
    private RotatingLauncher launcher;
    private Limelight3A limelight;
    private boolean isBlue = true;
    private int selectedPipeline = -1;
    private int ballCount = 8;
    private boolean shootFirst = true;
    private boolean lastCountLeft;
    private boolean lastCountRight;
    private boolean shootingActive;
    private boolean collectionActive;
    private double fixedFeedVelocity = Double.NaN;
    private long feedTargetLostSince;
    private long feedSpeedLowSince;
    private boolean feedPermitted;

    @Override
    public void runOpMode() {
        killMotors = false;
        drive = new Drive(hardwareMap, "Biobuzz", false);
        autoDrive = new AutonomousDriveCommands(this, drive);
        intake = new Intake(hardwareMap, DcMotor.Direction.FORWARD);
        intake.disableJamDetection();
        flowerPuller = new FlowerPuller(hardwareMap);
        launcher = new RotatingLauncher(hardwareMap, "Pollen", null, null);
        launcher.setFeedIntake(intake);
        limelight = hardwareMap.tryGet(Limelight3A.class, "Limelight");
        if (limelight != null) {
            limelight.setPollRateHz(100);
            limelight.start();
        }
        changeTeam(true);
        autoDrive.setPeriodicUpdate(this::updateNonShootingAim);

        TelemetryDashboard dashboard = new TelemetryDashboard(telemetry);
        while (!isStarted() && !isStopRequested()) {
            if (gamepad1.x) changeTeam(true);
            if (gamepad1.b) changeTeam(false);
            if (gamepad1.y) tipConfidence = TipConfidence.OPTIMISTIC;
            if (gamepad1.a) tipConfidence = TipConfidence.PESSIMISTIC;
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

            dashboard.addStatus(isBlue, drive.getDriveMode(), killMotors);
            telemetry.addData("Program", (shootFirst ? "Shoot First" : "Shoot Second")
                    + "          (d-pad up / down)");
            telemetry.addData("Balls", ballCount + "          (d-pad left / right)");
            /*telemetry.addData("Tip Confidence", tipConfidence == TipConfidence.OPTIMISTIC
                    ? "Optimistic          (Y optimistic / A pessimistic)"
                    : "Pessimistic          (Y optimistic / A pessimistic)");
            */telemetry.addLine();
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
            // Both programs clear the wall before aiming or running the route.
            if (ballCount == 0) {
                telemetry.addLine("0 balls selected; shooting skipped.");
                telemetry.update();
                return;
            }

            if (!intake.hasHopper() || !launcher.hasLoader()) {
                telemetry.addLine("Cannot feed: Hopper or Loader is not configured.");
                telemetry.update();
                return;
            }

            if (!shootBalls(ballCount)) return;

            if (!flowerPuller.isConfigured()) {
                telemetry.addLine("FlowerPuller missing; collection skipped.");
                telemetry.update();
                return;
            }

            if (shootFirst) {
                if (!strafeRight(2.85) || !forward(0.35)) return;
            } else {
                if (!strafeRight(1.5) || !turn(-90) || !strafeRight(1.3)) return;
            }

            flowerPuller.setDown();
            if (!waitForDuration(FLOWER_PULLER_MOVE_MS, "Lowering FlowerPuller", false)) return;
            collectionActive = true;
            intake.spin(true);
            if (!collectionForward()) return;
            if (!waitForDuration(FLOWER_COLLECTION_PAUSE_MS, "Collecting pollen", false)) return;
            if (!backward(0.5)) return;
            flowerPuller.setUp();
            if (!waitForDuration(FLOWER_PULLER_MOVE_MS, "Raising FlowerPuller", false)) return;

            if (shootFirst) {
                if (!strafeRight(3.8)) return;
            } else {
                if (!turn(90) || !backward(2.4)) return;
            }

            if (false) {
                collectionActive = false;
                intake.spin(false);
                if (tipConfidence == TipConfidence.OPTIMISTIC) {
                    if (!strafeRight(OPTIMISTIC_CROSS_FIELD_STRAFE_FEET) || !turn(180) || !strafeLeft(1.5))
                        return;
                }
            }

            //if (!shootBalls(ballCount)) return;

            if (shootFirst) {
                if (!turn(180) || !backward(0.8)) return;
            }

            drive.imu.resetYaw();

            telemetry.addLine("Autonomous route complete");
            telemetry.update();
        } finally {
            collectionActive = false;
            intake.onLoaderRunning(false);
            intake.spin(false);
            launcher.setLoaderRunning(false);
            launcher.stopLauncher();
            launcher.stopTurret();
            flowerPuller.setUp();
            autoDrive.stop();
        }
    }

    private boolean shootBalls(int count) {
        fixedFeedVelocity = Double.NaN;
        launcher.startAutonomousAiming();
        shootingActive = true;
        try {
            if (!waitForShotReady()) {
                telemetry.addLine("Tag alignment or shooting speed unavailable; feed skipped.");
                telemetry.update();
                return false;
            }

            fixedFeedVelocity = launcher.launcherVelocity;
            feedTargetLostSince = 0;
            feedSpeedLowSince = 0;
            feedPermitted = true;
            launcher.setAutonomousLoaderRunning(true);
            intake.setAutonomousFeedRunning(true);
            long feedDurationMs = Intake.HOPPER_LOADER_FEED_CYCLE_ENABLED
                    ? Intake.HOPPER_LOADER_INITIAL_PAUSE_MS + Intake.HOPPER_LOADER_CYCLE_MS * count
                            - Intake.HOPPER_LOADER_OFF_MS // Finish after the last loader pulse, before the final rest.
                    : FEED_MS_PER_BALL * count;
            if (!waitForDuration(feedDurationMs, "Feeding " + count + " balls", true)) return false;
            intake.onLoaderRunning(false);
            intake.spin(false);
            launcher.setLoaderRunning(false);
            return true;
        } finally {
            shootingActive = false;
            fixedFeedVelocity = Double.NaN;
            feedPermitted = false;
            intake.onLoaderRunning(false);
            intake.spin(false);
            launcher.setLoaderRunning(false);
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
                intake.spin(true);
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

    private boolean strafeLeft(double feet) {
        if (collectionActive) return collectionMove(-1, 0, feet * STRAFE_MS_PER_FOOT, 1.0);
        autoDrive.strafeLeft(feet * STRAFE_MS_PER_FOOT, 1);
        return opModeIsActive();
    }

    private boolean strafeRight(double feet) {
        autoDrive.strafeRight(feet * STRAFE_MS_PER_FOOT, 1);
        return opModeIsActive();
    }

    private boolean turn(double degrees) {
        return autoDrive.turnDegrees(degrees);
    }

    private boolean waitForShotReady() {
        long start = System.currentTimeMillis();
        while (opModeIsActive()
                && System.currentTimeMillis() - start < SHOOT_READY_TIMEOUT_MS) {
            updateShooting();
            if (launcher.readyToLaunch) return true;
            launcher.addCompetitionTelemetry(telemetry);
            telemetry.update();
            idle();
        }
        return false;
    }

    private void updateShooting() {
        launcher.updateAutonomousShooting(limelight == null ? null : limelight.getLatestResult(),
                isBlue, fixedFeedVelocity);
    }

    private void updateNonShootingAim() {
        if (!shootingActive) {
            launcher.updateVisibleTagTracking(limelight == null ? null : limelight.getLatestResult(), isBlue);
        }
    }

    private void updateFeedReadiness() {
        updateShooting();
        long now = System.currentTimeMillis();
        boolean targetVisible = launcher.isAimingTargetVisible();
        double velocity = launcher.launcher.getVelocity();
        boolean speedLow = !Double.isFinite(velocity) || velocity < fixedFeedVelocity * FEED_MIN_SPEED_FRACTION;
        if (targetVisible) feedTargetLostSince = 0;
        else if (feedTargetLostSince == 0) feedTargetLostSince = now;
        if (!speedLow) feedSpeedLowSince = 0;
        else if (feedSpeedLowSince == 0) feedSpeedLowSince = now;

        if (killMotors || feedTargetLostSince != 0 && now - feedTargetLostSince >= FEED_TARGET_LOSS_GRACE_MS
                || feedSpeedLowSince != 0 && now - feedSpeedLowSince >= FEED_SPEED_DROP_GRACE_MS) {
            feedPermitted = false;
        } else if (!feedPermitted && targetVisible && Double.isFinite(velocity)
                && velocity >= fixedFeedVelocity * FEED_RESUME_SPEED_FRACTION) {
            feedPermitted = true;
        }
    }

    private boolean waitForDuration(long durationMs, String phase, boolean pulseHopper) {
        long lastUpdate = System.currentTimeMillis();
        long lastReady = lastUpdate;
        long elapsedMs = 0;
        while (opModeIsActive() && elapsedMs < durationMs) {
            if (shootingActive) {
                updateFeedReadiness();
                launcher.setAutonomousLoaderRunning(feedPermitted);
            } else {
                updateNonShootingAim();
            }
            long now = System.currentTimeMillis();
            if (pulseHopper) {
                intake.setAutonomousFeedRunning(feedPermitted);
                if (feedPermitted) {
                    elapsedMs += now - lastUpdate;
                    lastReady = now;
                } else {
                    intake.runHopper(false);
                    if (now - lastReady >= SHOOT_READY_TIMEOUT_MS) {
                        telemetry.addLine("Tag alignment or shooting speed lost; feed stopped.");
                        telemetry.update();
                        return false;
                    }
                }
            } else {
                elapsedMs += now - lastUpdate;
            }
            lastUpdate = now;
            if (collectionActive && !killMotors) intake.spin(true);
            intake.updateHopperStall();
            telemetry.addData("Autonomous", phase);
            if (intake.isHopperStalled()) telemetry.addLine("Hopper stalled; feed stopped");
            if (shootingActive) {
                launcher.addCompetitionTelemetry(telemetry);
            } else {
                telemetry.addData("Flywheel", "%.0f / %.0f ticks/s",
                        launcher.launcher.getVelocity(), launcher.launcherVelocity);
            }
            telemetry.update();
            idle();
        }
        return opModeIsActive();
    }

    private void changeTeam(boolean blue) {
        int pipeline = blue ? 2 : 1;
        if (selectedPipeline == pipeline) return;
        isBlue = blue;
        selectedPipeline = pipeline;
        telemetry.speak(blue ? "Blue team" : "Red team");
        if (limelight != null) limelight.pipelineSwitch(pipeline);
    }
}
