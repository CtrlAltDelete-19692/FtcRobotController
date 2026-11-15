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
    private Limelight3A limelight;
    private boolean isBlue = true;
    private int selectedPipeline = -1;
    private boolean lastKillButton;
    private boolean lastDebugButton;
    private boolean debugTelemetry;

    @Override
    public void runOpMode() {
        killMotors = false;
        // Initialize Subsystems
        drive = new Drive(hardwareMap, "Biobuzz", false);
        drive.setResetYawOnRightBumper(false);
        intake = new Intake(hardwareMap, DcMotor.Direction.REVERSE);
        intake.enableJamRecovery();
        pollenLauncher = new RotatingLauncher(hardwareMap, "Pollen", null, null);
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
            pollenLauncher.updateEnabled(gamepad1);

            dashboard.addStatus(isBlue, drive.getDriveMode(), killMotors);
            telemetry.addLine("INIT: X Blue / B Red");
            telemetry.addData("Y Intake", intake.isEnabled() ? "ON" : "OFF");
            telemetry.addData("LB Loader", pollenLauncher.isLoaderEnabled() ? "ON" : "OFF");
            telemetry.addData("RB Flywheel", pollenLauncher.isFlywheelEnabled() ? "ON" : "OFF");
            telemetry.addLine("Flywheel speed: gamepad 2 LB / RB");
            dashboard.update();
        }

        if (isStopRequested()) return;
        waitForStart();
        telemetry.clearAll();
        lastKillButton = gamepad1.x;
        lastDebugButton = gamepad1.b;

        drive.imu.resetYaw();
        pollenLauncher.startHoming();

        while (opModeIsActive()) {
            if (gamepad1.x && !lastKillButton) toggleMotors();
            lastKillButton = gamepad1.x;
            if (gamepad1.b && !lastDebugButton) debugTelemetry = !debugTelemetry;
            lastDebugButton = gamepad1.b;

            // Run update() for all subsystems
            LLResult result = limelight == null ? null : limelight.getLatestResult();
            drive.update(gamepad1, gamepad2);
            intake.update(gamepad1, gamepad2);
            pollenLauncher.update(gamepad1, gamepad2, result, isBlue);
            //nectarLauncher.update(gamepad1);

            // Telemetry
            dashboard.addStatus(isBlue, drive.getDriveMode(), killMotors);
            addCompetitionTelemetry();
            if (debugTelemetry) {
                telemetry.addLine();
                telemetry.addLine("🔎 DEBUG - B to hide");
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
        String intakeStatus = killMotors || !intake.isEnabled() ? "❌ Intake" :
                intake.isReversing() ? "↩ Intake" : "✅ Intake";
        String flywheelStatus = !killMotors && pollenLauncher.isFlywheelEnabled() ?
                "✅ Flywheel" : "❌ Flywheel";
        String loaderStatus = !killMotors && pollenLauncher.isLoaderEnabled() ?
                pollenLauncher.hasLoader() ? "✅ Loader" : "⚠ Loader" : "❌ Loader";
        telemetry.addLine(intakeStatus + "      " + flywheelStatus + "      " + loaderStatus);
        pollenLauncher.addCompetitionTelemetry(telemetry);
    }

    private void changeTeam(boolean blue) {
        int pipeline = blue ? 2 : 1;
        if (selectedPipeline == pipeline) return;

        isBlue = blue;
        selectedPipeline = pipeline;
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
        }
    }
}
