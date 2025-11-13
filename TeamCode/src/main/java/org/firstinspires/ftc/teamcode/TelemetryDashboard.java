package org.firstinspires.ftc.teamcode;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;

public class TelemetryDashboard {

    private final Telemetry telemetry;
    private final Hardware hw;

    private boolean debugEnabled = true;
    private boolean speechEnabled = true;

    private String lastSpokenMode = "";

    public TelemetryDashboard(Telemetry telemetry, Hardware hardware) {
        this.telemetry = telemetry;
        this.hw = hardware;

        telemetry.setMsTransmissionInterval(50);
    }

    public void update(int teamTagId, Drive drive, Launcher launcher) {
        String teamColor = "Unknown Team";
        if (teamTagId == 20) {
            teamColor = "\uD83D\uDFE6 Blue";
        } else if (teamTagId == 24) {
            teamColor = "\uD83D\uDFE5 Red";
        }

        String driveMode = "Unknown Drive Mode";
        if (drive.getDriveMode() == Drive.DriveMode.FIELD_CENTRIC) {
            driveMode = "\uD83E\uDDED " + drive.getDriveMode();
        } else if(drive.getDriveMode() == Drive.DriveMode.MANUAL) {
            driveMode = "\uD83C\uDFAE " + drive.getDriveMode();
        }

        String autoHoming = "Auto-Aim";
        if (drive.autoCenterWithLauncher) {
            autoHoming = "\uD83D\uDFE2 " + autoHoming;
        } else {
            autoHoming = "⚪ " + autoHoming;
        }

        telemetry.addLine(teamColor + "      " + driveMode + "      " + autoHoming);

        if (debugEnabled) {
            telemetry.addLine();
            telemetry.addLine("=== Launcher ===");
            //telemetry.addData("PIDF", hw.launcher.getPIDFCoefficients(DcMotorEx.RunMode.RUN_USING_ENCODER));
            telemetry.addData("Velocity", "%.0f", hw.launcher.getVelocity());
            telemetry.addLine(String.format("%-10s %-10s %-10s", "Ticks", "Manual Δ", "Tag Δ"));
            telemetry.addLine(String.format("%-10.0f %-+15.0f %-+15.0f", launcher.launcherVelocity,  (double)launcher.lvManualAdjustment, (double)launcher.lvGoalDistanceAdjustment));

            telemetry.addLine();
            //telemetry.addLine("=== Tags ===");
            LLResult res = hw.limelight != null ? hw.limelight.getLatestResult() : null;
            //telemetry.addData("LL Valid Result", res != null && res.isValid());
            //telemetry.addData("LL Fiducial Count", (res != null && res.isValid()) ? res.getFiducialResults().size() : 0);

            int pipeline = res != null ? res.getPipelineIndex() : -1;
            //telemetry.addData("Pipeline", pipeline);
            if (hw.aprilTag.tagSeen) {
                //telemetry.addData("Tag X, Z", "%.2f, %.2f", hw.aprilTag.x, hw.aprilTag.z);
                telemetry.addLine(String.format("Tag found ✅, distance %.2f meters", hw.aprilTag.z));
            } else {
                telemetry.addLine("Tag not found ❌");
            }

            telemetry.addLine();
            telemetry.addLine("=== Viper Slides ===");
            telemetry.addData("Slides L / R", "%d / %d", hw.leftViperSlideMotor.getCurrentPosition(), hw.rightViperSlideMotor.getCurrentPosition());

            telemetry.addLine();
            telemetry.addLine("=== IMU ===");
            double headingDeg = hw.imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES);
            telemetry.addData("Heading", "%.1f deg", headingDeg);
        }

        if (speechEnabled) {
            String modeName = drive.getDriveMode().name().replace("_", " ").toLowerCase();
            if (!modeName.equals(lastSpokenMode)) {
                telemetry.speak(modeName);
                lastSpokenMode = modeName;
            }
        }

        telemetry.update();
    }
}
