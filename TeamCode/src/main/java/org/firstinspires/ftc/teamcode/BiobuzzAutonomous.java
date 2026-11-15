package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.hardware.DcMotor;

@Autonomous(name = "BiobuzzAutonomous")
public class BiobuzzAutonomous extends CtrlAltDelOpMode {
    private static final long FLYWHEEL_SPINUP_TIMEOUT_MS = 5000;
    private static final long LOADER_LEAD_MS = 150;
    private static final long FEED_MS_PER_BALL = 1000; // Tune on the robot; balls are not counted yet.
    private static final long LOADER_CLEAR_MS = 400;

    private Drive drive;
    private AutonomousDriveCommands autoDrive;
    private Intake intake;
    private RotatingLauncher launcher;
    private Limelight3A limelight;
    private boolean isBlue = true;
    private int selectedPipeline = -1;
    private int ballCount = 4;

    @Override
    public void runOpMode() {
        killMotors = false;
        drive = new Drive(hardwareMap, "Biobuzz", false);
        autoDrive = new AutonomousDriveCommands(this, drive);
        intake = new Intake(hardwareMap, DcMotor.Direction.REVERSE);
        launcher = new RotatingLauncher(hardwareMap, "Pollen", null, null);
        limelight = hardwareMap.tryGet(Limelight3A.class, "Limelight");
        if (limelight != null) limelight.start();
        changeTeam(true);

        TelemetryDashboard dashboard = new TelemetryDashboard(telemetry);
        while (!isStarted() && !isStopRequested()) {
            if (gamepad1.x) changeTeam(true);
            if (gamepad1.b) changeTeam(false);
            if (gamepad1.dpad_left) ballCount = 3;
            if (gamepad1.dpad_right) ballCount = 4;

            dashboard.addStatus(isBlue, drive.getDriveMode(), killMotors);
            telemetry.addLine("X Blue / B Red     D-pad Left 3 / Right 4 balls");
            telemetry.addData("Feed target", ballCount + " balls (timed)");
            telemetry.addData("Hopper", intake.hasHopper() ? "✅ Configured" : "⚠ Missing");
            telemetry.addData("Loader", launcher.hasLoader() ? "✅ Configured" : "⚠ Missing");
            telemetry.addLine("Place robot aimed at the HIVE; this program does not drive.");
            dashboard.update();
        }

        if (isStopRequested()) return;
        waitForStart();
        telemetry.clearAll();

        try {
            drive.imu.resetYaw();
            autoDrive.stop();
            if (!intake.hasHopper() || !launcher.hasLoader()) {
                telemetry.addLine("Cannot feed: Hopper or Loader is not configured.");
                telemetry.update();
                return;
            }

            launcher.setAutonomousFlywheelVelocity(RotatingLauncher.LAUNCHER_BASE_TICKS);
            if (!waitForFlywheel()) {
                telemetry.addLine("Flywheel did not reach shooting speed; feed skipped.");
                telemetry.update();
                return;
            }

            launcher.setLoaderRunning(true);
            if (!waitForDuration(LOADER_LEAD_MS, "Starting Loader")) return;
            intake.runHopper(true);
            if (!waitForDuration(FEED_MS_PER_BALL * ballCount, "Feeding " + ballCount + " balls")) return;
            intake.runHopper(false);
            if (!waitForDuration(LOADER_CLEAR_MS, "Clearing Loader")) return;

            telemetry.addLine("Timed feed complete");
            telemetry.update();
        } finally {
            intake.runHopper(false);
            launcher.setLoaderRunning(false);
            launcher.stopLauncher();
            launcher.stopTurret();
            autoDrive.stop();
        }
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

    private boolean waitForDuration(long durationMs, String phase) {
        long start = System.currentTimeMillis();
        while (opModeIsActive() && System.currentTimeMillis() - start < durationMs) {
            telemetry.addData("Autonomous", phase);
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
        if (limelight != null) limelight.pipelineSwitch(pipeline);
    }
}
