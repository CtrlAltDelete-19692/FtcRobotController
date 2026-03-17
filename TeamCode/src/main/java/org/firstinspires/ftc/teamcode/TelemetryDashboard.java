package org.firstinspires.ftc.teamcode;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;

public class TelemetryDashboard {

    private final Telemetry telemetry;
    private Drive drive;
    private Launcher launcher;
    private AprilTag aprilTag;
    private Intake intake;
    private Slides slides;

    private boolean debugEnabled = true;
    private boolean speechEnabled = true;

    private String lastSpokenMode = "";

    public TelemetryDashboard(Telemetry telemetry, Drive drive, Launcher launcher, AprilTag aprilTag, Intake intake, Slides slides) {
        this.telemetry = telemetry;
        telemetry.setMsTransmissionInterval(50);

        this.drive = drive;
        this.launcher = launcher;
        this.aprilTag = aprilTag;
        this.intake = intake;
        this.slides = slides;
    }

    public void update(int teamTagId) {
        String teamColor = "Unknown Team";
        if (teamTagId == 20) {
            teamColor = "\uD83D\uDFE6 Blue";
        } else if (teamTagId == 24) {
            teamColor = "\uD83D\uDFE5 Red";
        }

        String driveMode = "Unknown Drive Mode";
        if (drive != null) {
            if (drive.getDriveMode() == Drive.DriveMode.FIELD_CENTRIC) {
                driveMode = "\uD83E\uDDED " + drive.getDriveMode();
            } else if (drive.getDriveMode() == Drive.DriveMode.MANUAL) {
                driveMode = "\uD83C\uDFAE " + drive.getDriveMode();
            }
        }

        String motors = "Motors";
        if (CtrlAltDelOpMode.killMotors) {
            motors = "❌ " + motors;
        } else {
            motors = "✅ " + motors;
        }

        telemetry.addLine(teamColor + "      " + driveMode + "      " + motors);
        telemetry.addLine();

        if (debugEnabled) {
            if (launcher.launcher != null) {
                String launcherIcon = "\uD83C\uDF00";
                if (launcher.launcherVelocity <= 0) {
                    launcherIcon = "";
                } else if (launcher.readyToLaunch) {
                    launcherIcon = "✅";
                }
                telemetry.addLine(String.format("Launcher: %.0f %s", launcher.launcher.getVelocity(), launcherIcon));
                telemetry.addLine(String.format("Target      %.0f (Tag: %d, Manual: %d)", launcher.launcherVelocity, launcher.lvGoalDistanceAdjustment, launcher.lvManualAdjustment));
            }

            if (launcher.loader != null) {
                telemetry.addLine(String.format("Loader: %.0f%%", launcher.loader.getPower() * 100));
            }

            if (intake != null) {
                telemetry.addLine(String.format("Pickup: %.1f%%", intake.pickupMotor.getPower() * 100));
                telemetry.addLine();
            }

            if (aprilTag != null) {
                LLResult res = aprilTag.limelight != null ? aprilTag.limelight.getLatestResult() : null;
                if (aprilTag.tagSeen) {
                    //telemetry.addData("Tag X, Z", "%.2f, %.2f", aprilTag.x, aprilTag.z);
                    telemetry.addLine(String.format("Tag found ✅, %.2f ft", aprilTag.zFeet));
                } else {
                    telemetry.addLine("Tag not found ❌");
                }
                telemetry.addLine();
            }

            if (slides != null) {
                telemetry.addData("Slides L / R", "%d / %d", slides.leftViperSlideMotor.getCurrentPosition(), slides.rightViperSlideMotor.getCurrentPosition());
            }

            if (drive != null) {
                double headingDeg = drive.imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES);
                telemetry.addData("Heading", "%.1f deg", headingDeg);

                if (speechEnabled) {
                    String modeName = drive.getDriveMode().name().replace("_", " ").toLowerCase();
                    if (!modeName.equals(lastSpokenMode)) {
                        telemetry.speak(modeName);
                        lastSpokenMode = modeName;
                    }
                }
            }
        }

        telemetry.update();
    }
}
