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

    @Override
    public void runOpMode() {
        // Initialize Subsystems
        drive = new Drive(hardwareMap, "Biobuzz", false);
        drive.setResetYawOnRightBumper(false);
        intake = new Intake(hardwareMap, DcMotor.Direction.REVERSE);
        pollenLauncher = new RotatingLauncher(hardwareMap, "Pollen", null, null);
        //nectarLauncher = new RotatingLauncher(hardwareMap, "Nectar", null, null);
        limelight = hardwareMap.tryGet(Limelight3A.class, "Limelight");
        if (limelight != null) {
            limelight.setPollRateHz(100);
            limelight.start();
        }
        changeTeam(true);

        TelemetryDashboard dashboard = new TelemetryDashboard(telemetry);

        while (!isStarted()) {
            if (gamepad1.x) changeTeam(true);
            if (gamepad1.b) changeTeam(false);

            // Telemetry Setup
            dashboard.addStatus(isBlue, drive.getDriveMode(), killMotors);
            pollenLauncher.addTelemetry(telemetry);
            //nectarLauncher.addTelemetry(telemetry);
            addLimelightTelemetry();
            intake.addTelemetry(telemetry);
            drive.addTelemetry(telemetry);
            dashboard.update();
        }

        waitForStart();

        drive.imu.resetYaw();
        pollenLauncher.startHoming();

        while (opModeIsActive()) {
            // Run update() for all subsystems
            drive.update(gamepad1, gamepad2);
            intake.update(gamepad1, gamepad2);
            pollenLauncher.update(gamepad1);
            //nectarLauncher.update(gamepad1);

            // Telemetry
            dashboard.addStatus(isBlue, drive.getDriveMode(), killMotors);
            pollenLauncher.addTelemetry(telemetry);
            //nectarLauncher.addTelemetry(telemetry);
            addLimelightTelemetry();
            intake.addTelemetry(telemetry);
            drive.addTelemetry(telemetry);
            dashboard.update();
        }
        pollenLauncher.stopTurret();
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

    private void addLimelightTelemetry() {
        if (limelight == null) {
            telemetry.addData("Limelight", "Not configured");
            return;
        }

        LLResult result = limelight.getLatestResult();
        if (result == null || !result.isValid()) {
            telemetry.addData("Limelight", "No valid result");
            return;
        }

        List<LLResultTypes.FiducialResult> tags = result.getFiducialResults();
        if (tags == null || tags.isEmpty()) {
            telemetry.addData("Limelight", "No AprilTags detected");
            return;
        }

        telemetry.addData("Limelight selected tx", "%.1f deg", result.getTx());
        for (LLResultTypes.FiducialResult tag : tags) {
            telemetry.addData("Tag " + tag.getFiducialId() + " tx", "%.1f deg",
                    tag.getTargetXDegrees());
        }
    }
}
