package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;


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

    public void setDebugEnabled(boolean enabled) {
        this.debugEnabled = enabled;
    }

    public void setSpeechEnabled(boolean enabled) {
        this.speechEnabled = enabled;
    }

    public void update(Drive drive) { // Test: Does telemetry work? Try debug mode. Audio?
        Orientation angles = hw.imu.getAngularOrientation(
                AxesReference.INTRINSIC,
                AxesOrder.ZYX,
                AngleUnit.DEGREES
        );
        double headingDeg = angles.firstAngle;

        telemetry.addData("Drive Mode", drive.getDriveMode());
        telemetry.addData("Heading", "%.1f deg", headingDeg);

        if (debugEnabled) {
            telemetry.addData("Status", hw.imu.getSystemStatus().toString());
            telemetry.addData("Calib", hw.imu.getCalibrationStatus().toString());
            
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
