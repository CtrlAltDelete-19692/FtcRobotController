package org.firstinspires.ftc.teamcode;

import org.firstinspires.ftc.robotcore.external.Telemetry;

public class TelemetryDashboard {

    private final Telemetry telemetry;
    private String lastSpokenMode = "";

    public TelemetryDashboard(Telemetry telemetry) {
        this.telemetry = telemetry;
        telemetry.setMsTransmissionInterval(50);
    }

    public void addStatus(int teamTagId, Drive.DriveMode driveMode, boolean killMotors) {
        String teamColor = "Unknown Team";
        if (teamTagId == 20) {
            teamColor = "\uD83D\uDFE6 Blue";
        } else if (teamTagId == 24) {
            teamColor = "\uD83D\uDFE5 Red";
        }

        String modeLabel = "Unknown Drive Mode";
        if (driveMode == Drive.DriveMode.FIELD_CENTRIC) {
            modeLabel = "\uD83E\uDDED " + driveMode;
        } else if (driveMode == Drive.DriveMode.MANUAL) {
            modeLabel = "\uD83C\uDFAE " + driveMode;
        }

        telemetry.addLine(teamColor + "      " + modeLabel + "      "
                + (killMotors ? "❌ Motors" : "✅ Motors"));
        telemetry.addLine();

        if (driveMode != null) {
            String modeName = driveMode.name().replace("_", " ").toLowerCase();
            if (!modeName.equals(lastSpokenMode)) {
                telemetry.speak(modeName);
                lastSpokenMode = modeName;
            }
        }
    }

    public void update() {
        telemetry.update();
    }
}
