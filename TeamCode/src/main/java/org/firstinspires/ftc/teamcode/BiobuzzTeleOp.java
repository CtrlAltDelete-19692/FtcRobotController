package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

import java.util.List;

@TeleOp(name = "BiobuzzTeleOp")

public class BiobuzzTeleOp extends CtrlAltDelOpMode {
    public Drive drive;
    private RotatingLauncher pollenLauncher;
    private RotatingLauncher nectarLauncher;
    private Intake intake;
    private BallPurge ballPurge;
    private FlowerPuller flowerPuller;
    private Limelight3A limelight;
    private boolean isBlue = true;
    private int selectedPipeline = -1;
    private boolean lastYawResetButton;
    private boolean lastDebugButton;
    private boolean debugTelemetry;

    @Override
    public void runOpMode() {
        killMotors = false;
        // Initialize Subsystems
        drive = new Drive(hardwareMap, "Biobuzz", false);
        drive.setResetYawOnRightBumper(false);
        intake = new Intake(hardwareMap, DcMotor.Direction.FORWARD);
        intake.enableJamRecovery();
        flowerPuller = new FlowerPuller(hardwareMap);
        pollenLauncher = new RotatingLauncher(hardwareMap, "Pollen", null, null);
        pollenLauncher.setFeedIntake(intake);
        ballPurge = new BallPurge(intake, pollenLauncher);
        //nectarLauncher = new RotatingLauncher(hardwareMap, "Nectar", null, null);
        limelight = hardwareMap.tryGet(Limelight3A.class, "Limelight");
        if (limelight != null) {
            limelight.setPollRateHz(100);
            limelight.start();
        }
        changeTeam(true);

        TelemetryDashboard dashboard = new TelemetryDashboard(telemetry);

        while (!isStarted() && !isStopRequested()) {
            if (gamepad1.x) changeTeam(true);
            if (gamepad1.b) changeTeam(false);
            intake.updateEnabled(gamepad1, gamepad2);
            updateDebugTelemetry();

            dashboard.addStatus(isBlue, drive.getDriveMode());
            telemetry.addLine("Flywheel speed: gamepad 2 LB / RB");
            if (debugTelemetry) {
                telemetry.addData("Intake (Y)", intake.isEnabled() ? "ON" : "OFF");
                telemetry.addLine("Loader (gamepad 2 A/B); purge: hold gamepad 2 LT");
                telemetry.addData("FlowerPuller", flowerPuller.isConfigured() ? "✅ Configured" : "⚠ Missing");
            }
            dashboard.update();
        }

        if (isStopRequested()) return;
        waitForStart();
        flowerPuller.setUp();
        telemetry.clearAll();
        lastYawResetButton = gamepad1.left_bumper;
        lastDebugButton = gamepad2.back || gamepad2.share;

        drive.imu.resetYaw();
        pollenLauncher.startHoming();

        while (opModeIsActive()) {
            if (gamepad1.left_bumper && !lastYawResetButton) drive.imu.resetYaw();
            lastYawResetButton = gamepad1.left_bumper;
            updateDebugTelemetry();

            // Run update() for all subsystems
            LLResult result = limelight == null ? null : limelight.getLatestResult();
            ballPurge.setRunning(gamepad2.left_trigger > TRIGGER_DEADZONE);
            drive.update(gamepad1, gamepad2);
            intake.setManualReverse(gamepad1.x || gamepad2.x);
            intake.update(gamepad1, gamepad2);
            flowerPuller.update(gamepad1);
            pollenLauncher.update(gamepad1, gamepad2, result, isBlue);
            intake.updateHopperStall();
            //nectarLauncher.update(gamepad1);

            // Telemetry
            dashboard.addStatus(isBlue, drive.getDriveMode());
            addCompetitionTelemetry();
            if (debugTelemetry) {
                telemetry.addLine();
                telemetry.addLine("SHOOTER / TURRET");
                pollenLauncher.addTelemetry(telemetry);
                telemetry.addLine("LIMELIGHT");
                addLimelightTelemetry(result);
                telemetry.addLine("INTAKE");
                intake.addTelemetry(telemetry);
                telemetry.addLine("DRIVE");
                drive.addDebugTelemetry(telemetry);
            }
            dashboard.update();
        }
        ballPurge.setRunning(false);
        pollenLauncher.stopTurret();
    }

    private void addCompetitionTelemetry() {
        if (ballPurge.isRunning()) telemetry.addLine("↩ Clearing balls (gamepad 2 LT)");
        String intakeStatus = ballPurge.isRunning() ? "↩ Intake" :
                killMotors ? "❌ Intake (Y)" :
                        intake.isManualReversing() ? "↩ Intake (X held)" :
                !intake.isEnabled() ? "❌ Intake (Y)" :
                        intake.isIntakeFeedPauseActive() ? "⏸ Intake (feed pause)" :
                                intake.isReversing() ? "↩ Intake (Y)" : "✅ Intake (Y)";
        String flywheelStatus = killMotors ? "❌ Flywheel" :
                ballPurge.isRunning() ? "↩ Flywheel" : "✅ Flywheel";
        String loaderStatus = !pollenLauncher.hasLoader() ? "⚠ Loader (2: A/B)" :
                ballPurge.isRunning() ? "↩ Loader" :
                        pollenLauncher.isLoaderRunning() ? "✅ Loader (2: A/B)" :
                                intake.isLoaderFeedRequested() ? "⌛ Loader (pulse wait)" : "❌ Loader (2: A/B)";
        String hopperStatus = !intake.hasHopper() ? "⚠ Hopper" :
                ballPurge.isRunning() ? "↩ Hopper" :
                intake.isHopperStalled() ? "⚠ Hopper stalled" :
                        intake.isHopperLoaderOverride() && !intake.isEnabled() ? "✅ Hopper (Loader)" : "";
        telemetry.addLine(intakeStatus + "      " + flywheelStatus + "      " + loaderStatus);
        if (!hopperStatus.isEmpty()) telemetry.addLine(hopperStatus);
        pollenLauncher.addCompetitionTelemetry(telemetry);
    }

    private void updateDebugTelemetry() {
        boolean debugButton = gamepad2.back || gamepad2.share;
        if (debugButton && !lastDebugButton) debugTelemetry = !debugTelemetry;
        lastDebugButton = debugButton;
    }

    private void changeTeam(boolean blue) {
        int pipeline = blue ? 2 : 1;
        if (selectedPipeline == pipeline) return;

        isBlue = blue;
        selectedPipeline = pipeline;
        telemetry.speak(blue ? "Blue team" : "Red team");
        if (limelight != null) {
            limelight.pipelineSwitch(pipeline);
        }
    }

    private void addLimelightTelemetry(LLResult result) {
        if (limelight == null) {
            telemetry.addLine("Not configured");
            return;
        }

        if (result == null || !result.isValid()) {
            telemetry.addLine("No valid result");
            return;
        }

        List<LLResultTypes.FiducialResult> tags = result.getFiducialResults();
        if (tags == null || tags.isEmpty()) {
            telemetry.addLine("No AprilTags detected");
            return;
        }

        telemetry.addData("Selected tx", "%.1f deg", result.getTx());
        for (LLResultTypes.FiducialResult tag : tags) {
            telemetry.addData("Tag " + tag.getFiducialId() + " tx", "%.1f deg",
                    tag.getTargetXDegrees());
            telemetry.addData("Tag " + tag.getFiducialId() + " height", "%.1f in  %s (must be > %.0f)",
                    pollenLauncher.getTagHeightInches(tag),
                    pollenLauncher.isTagAboveMinimumHeight(tag) ? "Accepted" : "FILTERED OUT",
                    RotatingLauncher.MIN_TARGET_HEIGHT_INCHES);
        }
    }
}
