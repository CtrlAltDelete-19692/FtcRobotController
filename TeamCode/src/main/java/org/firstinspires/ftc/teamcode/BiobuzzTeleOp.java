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
    private FlowerPuller flowerPuller;
    private Limelight3A limelight;
    private boolean isBlue = true;
    private int selectedPipeline = -1;
    private boolean lastKillButton;
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

            dashboard.addStatus(isBlue, drive.getDriveMode(), killMotors);
            telemetry.addLine("Flywheel speed: gamepad 2 LB / RB");
            if (debugTelemetry) {
                telemetry.addData("Intake (Y)", intake.isEnabled() ? "ON" : "OFF");
                telemetry.addLine("Loader (A or B): hold A (ready + tag) / B (manual)");
                telemetry.addData("FlowerPuller", flowerPuller.isConfigured() ? "✅ Configured" : "⚠ Missing");
            }
            dashboard.update();
        }

        if (isStopRequested()) return;
        waitForStart();
        flowerPuller.setUp();
        telemetry.clearAll();
        lastKillButton = gamepad1.x;
        lastYawResetButton = gamepad1.left_bumper;
        lastDebugButton = gamepad2.x;

        drive.imu.resetYaw();
        pollenLauncher.startHoming();

        while (opModeIsActive()) {
            if (gamepad1.x && !lastKillButton) toggleMotors();
            lastKillButton = gamepad1.x;
            if (gamepad1.left_bumper && !lastYawResetButton) drive.imu.resetYaw();
            lastYawResetButton = gamepad1.left_bumper;
            updateDebugTelemetry();

            // Run update() for all subsystems
            LLResult result = limelight == null ? null : limelight.getLatestResult();
            drive.update(gamepad1, gamepad2);
            intake.update(gamepad1, gamepad2);
            flowerPuller.update(gamepad2);
            pollenLauncher.update(gamepad1, gamepad2, result, isBlue);
            intake.updateHopperStall();
            //nectarLauncher.update(gamepad1);

            // Telemetry
            dashboard.addStatus(isBlue, drive.getDriveMode(), killMotors);
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
        pollenLauncher.stopTurret();
    }

    private void addCompetitionTelemetry() {
        String intakeStatus = killMotors || !intake.isEnabled() ? "❌ Intake (Y)" :
                intake.isReversing() ? "↩ Intake (Y)" : "✅ Intake (Y)";
        String flywheelStatus = !killMotors ?
                "✅ Flywheel" : "❌ Flywheel";
        String loaderStatus = !pollenLauncher.hasLoader() ? "⚠ Loader (A or B)" :
                pollenLauncher.isLoaderRunning() ? "✅ Loader (A or B)" : "❌ Loader (A or B)";
        String hopperStatus = !intake.hasHopper() ? "⚠ Hopper" :
                intake.isHopperStalled() ? "⚠ Hopper stalled" :
                        intake.isHopperLoaderOverride() && !intake.isEnabled() ? "✅ Hopper (Loader)" : "";
        telemetry.addLine(intakeStatus + "      " + flywheelStatus + "      " + loaderStatus);
        if (!hopperStatus.isEmpty()) telemetry.addLine(hopperStatus);
        pollenLauncher.addCompetitionTelemetry(telemetry);
    }

    private void updateDebugTelemetry() {
        if (gamepad2.x && !lastDebugButton) debugTelemetry = !debugTelemetry;
        lastDebugButton = gamepad2.x;
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
