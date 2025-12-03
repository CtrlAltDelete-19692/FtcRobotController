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

        String motors = "Motors";
        if (hw.killMotors) {
            motors = "❌ " + motors;
        } else {
            motors = "✅ " + motors;
        }

        telemetry.addLine(teamColor + "      " + driveMode + "      " + motors);
        telemetry.addLine();

        if (debugEnabled) {
            String launcherIcon = "\uD83C\uDF00";
            if (launcher.launcherVelocity <= 0) {
                launcherIcon = "";
            } else if (launcher.readyToLaunch) {
                launcherIcon = "✅";
            }
            telemetry.addLine(String.format("Launcher: %.0f %s", hw.launcher.getVelocity(), launcherIcon));
            telemetry.addLine(String.format("Target      %.0f (Tag: %d, Manual: %d)", launcher.launcherVelocity, launcher.lvGoalDistanceAdjustment, launcher.lvManualAdjustment));
            telemetry.addLine();

            LLResult res = hw.limelight != null ? hw.limelight.getLatestResult() : null;
            int pipeline = res != null ? res.getPipelineIndex() : -1;
            //telemetry.addData("Pipeline", pipeline);
            if (hw.aprilTag.tagSeen) {
                //telemetry.addData("Tag X, Z", "%.2f, %.2f", hw.aprilTag.x, hw.aprilTag.z);
                telemetry.addLine(String.format("Tag found ✅, %.2f ft", hw.aprilTag.zFeet));
            } else {
                telemetry.addLine("Tag not found ❌");
            }
            telemetry.addLine();

            telemetry.addData("Slides L / R", "%d / %d", hw.leftViperSlideMotor.getCurrentPosition(), hw.rightViperSlideMotor.getCurrentPosition());

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
