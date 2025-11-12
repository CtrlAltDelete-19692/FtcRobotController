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
        String teamColor = "Unknown";
        if (teamTagId == 20) {
            teamColor = "\uD83D\uDFE6 Blue";
        } else if (teamTagId == 24) {
            teamColor = "\uD83D\uDFE5 Red";
        }
        telemetry.addData("Team Color", teamColor);

        telemetry.addData("Drive Mode", drive.getDriveMode());

        if (debugEnabled) {
            double headingDeg = hw.imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES);
            telemetry.addData("Heading", "%.1f deg", headingDeg);

            //telemetry.addData("PIDF", hw.launcher.getPIDFCoefficients(DcMotorEx.RunMode.RUN_USING_ENCODER));
            telemetry.addData("Launcher Velocity", "%.0f / %.0f", hw.launcher.getVelocity(), launcher.launcherVelocity);
            telemetry.addData("LV Add", "%d",launcher.add);

            if (hw.limelight != null && hw.l) {
                telemetry.addLine("Limelight Connected ✅");
                telemetry.addData("Device Name", "Limelight");
                //telemetry.addData("Poll Rate", hw.limelight.getPollRateHz());
            } else {
                telemetry.addLine("Limelight Not Connected ❌");
                telemetry.addLine("Check Robot Config name matches exactly: 'Limelight'");
            }

            LLResult result = hw.limelight.getLatestResult();
            LLResult res = hw.limelight != null ? hw.limelight.getLatestResult() : null;
            telemetry.addData("LL Valid Result", res != null && res.isValid());
            telemetry.addData("LL Fiducial Count",
                    (res != null && res.isValid()) ? res.getFiducialResults().size() : 0);


            telemetry.addData("Tag Seen", hw.aprilTag.tagSeen);
            telemetry.addData("Tag X, Z", "%.2f, %.2f", hw.aprilTag.x, hw.aprilTag.z);
            telemetry.addData("LL Pipeline", result != null ? result.getPipelineIndex() : -1);

            telemetry.addData("Slides L / R", "%d / %d", hw.leftViperSlideMotor.getCurrentPosition(), hw.rightViperSlideMotor.getCurrentPosition());
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
