package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.hardware.DcMotor;

@Autonomous(name = "BiobuzzAutonomous")
public class BiobuzzAutonomous extends CtrlAltDelOpMode {
    private static final long FLYWHEEL_SPINUP_TIMEOUT_MS = 5000;
    private static final long FEED_MS_PER_BALL = 1000;
    private static final long LOADER_CLEAR_MS = 400;
    private static final long FORWARD_MS_PER_FOOT = 350; // Starting estimate from Decode; tune on Biobuzz.
    private static final long STRAFE_MS_PER_FOOT = 500;  // Starting estimate from Decode; tune on Biobuzz.
    private static final long FLOWER_PULLER_MOVE_MS = 800;

    private Drive drive;
    private AutonomousDriveCommands autoDrive;
    private Intake intake;
    private FlowerPuller flowerPuller;
    private RotatingLauncher launcher;
    private Limelight3A limelight;
    private boolean isBlue = true;
    private int selectedPipeline = -1;
    private int ballCount = 4;
    private boolean shootFirst = true;
    private boolean lastCountLeft;
    private boolean lastCountRight;

    @Override
    public void runOpMode() {
        killMotors = false;
        drive = new Drive(hardwareMap, "Biobuzz", false);
        autoDrive = new AutonomousDriveCommands(this, drive);
        intake = new Intake(hardwareMap, DcMotor.Direction.FORWARD);
        flowerPuller = new FlowerPuller(hardwareMap);
        launcher = new RotatingLauncher(hardwareMap, "Pollen", null, null);
        limelight = hardwareMap.tryGet(Limelight3A.class, "Limelight");
        if (limelight != null) limelight.start();
        changeTeam(true);

        TelemetryDashboard dashboard = new TelemetryDashboard(telemetry);
        while (!isStarted() && !isStopRequested()) {
            if (gamepad1.x) changeTeam(true);
            if (gamepad1.b) changeTeam(false);
            if (gamepad1.dpad_up) shootFirst = true;
            if (gamepad1.dpad_down) shootFirst = false;
            if (gamepad1.dpad_left && !lastCountLeft) ballCount--;
            if (gamepad1.dpad_right && !lastCountRight) ballCount++;
            lastCountLeft = gamepad1.dpad_left;
            lastCountRight = gamepad1.dpad_right;
            if (ballCount < 0) {
                ballCount = 0;
            } else if (ballCount > 6) {
                ballCount = 6;
            }

            dashboard.addStatus(isBlue, drive.getDriveMode(), killMotors);
            telemetry.addData("Route", shootFirst ? "Shoot First (D-pad up)" : "Shoot Second (D-pad down)");
            telemetry.addData("Balls", ballCount + " D-pad Left / Right");
            telemetry.addLine();
            telemetry.addData("Hopper", intake.hasHopper() ? "✅ Configured" : "⚠ Missing");
            telemetry.addData("Loader", launcher.hasLoader() ? "✅ Configured" : "⚠ Missing");
            telemetry.addData("FlowerPuller", flowerPuller.isConfigured() ? "✅ Configured" : "⚠ Missing");
            telemetry.addLine("Place robot aimed at the HIVE; check that the route is clear.");
            dashboard.update();
        }

        if (isStopRequested()) return;
        waitForStart();
        flowerPuller.setUp();
        telemetry.clearAll();

        try {
            if (ballCount == 0) {
                telemetry.addLine("0 balls selected; shooting skipped.");
                telemetry.update();
                return;
            }
            drive.imu.resetYaw();
            autoDrive.stop();

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
                if (!forward(3.5) || !turn(-90) || !forward(4)) return;
            } else {
                if (!forward(2) || !turn(90) || !forward(1) || !turn(90)) return;
            }

            flowerPuller.setDown();
            if (!waitForDuration(FLOWER_PULLER_MOVE_MS, "Lowering FlowerPuller", false)) return;
            intake.spin(true);
            if (!forward(shootFirst ? 2 : 2.5) || !backward(1)) return;
            flowerPuller.setUp();
            intake.spin(false);
            if (!waitForDuration(FLOWER_PULLER_MOVE_MS, "Raising FlowerPuller", false)) return;
            if (!turn(180)) return;

            if (shootFirst) {
                if (!strafeRight(2) || !forward(4) || !turn(-90) || !backward(1.5)) return;
            } else {
                if (!strafeLeft(1)) return;
            }

            if (!shootBalls(ballCount)) return;

            telemetry.addLine("Autonomous route complete");
            telemetry.update();
        } finally {
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
        launcher.setAutonomousFlywheelVelocity(RotatingLauncher.LAUNCHER_BASE_TICKS);
        if (!waitForFlywheel()) {
            telemetry.addLine("Flywheel did not reach shooting speed; feed skipped.");
            telemetry.update();
            return false;
        }

        launcher.setLoaderRunning(true);
        intake.onLoaderRunning(true);
        long feedDurationMs = Intake.HOPPER_LOADER_FEED_CYCLE_ENABLED
                ? Intake.HOPPER_LOADER_INITIAL_PAUSE_MS + Intake.HOPPER_LOADER_CYCLE_MS * count
                : FEED_MS_PER_BALL * count;
        if (!waitForDuration(feedDurationMs, "Feeding " + count + " balls", true)) return false;
        intake.onLoaderRunning(false);
        intake.runHopper(false);
        if (!waitForDuration(LOADER_CLEAR_MS, "Clearing Loader", false)) return false;
        launcher.setLoaderRunning(false);
        return true;
    }

    private boolean forward(double feet) {
        autoDrive.forward(feet * FORWARD_MS_PER_FOOT, 1);
        return opModeIsActive();
    }

    private boolean backward(double feet) {
        autoDrive.backward(feet * FORWARD_MS_PER_FOOT, 1);
        return opModeIsActive();
    }

    private boolean strafeLeft(double feet) {
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

    private boolean waitForFlywheel() {
        long start = System.currentTimeMillis();
        while (opModeIsActive() && !launcher.upToSpeed()
                && System.currentTimeMillis() - start < FLYWHEEL_SPINUP_TIMEOUT_MS) {
            telemetry.addData("Flywheel", "%.0f / %.0f ticks/s",
                    launcher.launcher.getVelocity(), launcher.launcherVelocity);
            telemetry.update();
            idle();
        }
        return opModeIsActive() && launcher.upToSpeed();
    }

    private boolean waitForDuration(long durationMs, String phase, boolean pulseHopper) {
        long start = System.currentTimeMillis();
        while (opModeIsActive() && System.currentTimeMillis() - start < durationMs) {
            if (pulseHopper) intake.onLoaderRunning(true);
            intake.updateHopperStall();
            telemetry.addData("Autonomous", phase);
            if (intake.isHopperStalled()) telemetry.addLine("Hopper stalled; feed stopped");
            telemetry.addData("Flywheel", "%.0f / %.0f ticks/s",
                    launcher.launcher.getVelocity(), launcher.launcherVelocity);
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
