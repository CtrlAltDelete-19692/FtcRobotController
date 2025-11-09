package org.firstinspires.ftc.teamcode;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

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

    public void update(Drive drive, double launcherVelocity, double distance) { // Test: Does telemetry work? Try debug mode. Audio?
        telemetry.addData("Drive Mode", drive.getDriveMode());

        double headingDeg = hw.imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES);
        telemetry.addData("Heading", "%.1f deg", headingDeg);
        telemetry.addData("Launcher Velocity", "% / %", launcherVelocity, hw.launcher.getVelocity()); // Test: Ensure launcher speeds work against tag distance
        telemetry.addData("Distance", distance); // Test: for correctness

        if (debugEnabled) {
            double[] p = drive.getWheelPowers();
            telemetry.addData("LF / RF", "%.2f / %.2f", p[0], p[1]);
            telemetry.addData("LB / RB", "%.2f / %.2f", p[2], p[3]);
        }

        if (speechEnabled) {
            String modeName = drive.getDriveMode().name().replace("_", " ").toLowerCase();
            if (!modeName.equals(lastSpokenMode)) {
                telemetry.speak("Drive mode " + modeName);
                lastSpokenMode = modeName;
            }
        }

        telemetry.update();
    }
}
