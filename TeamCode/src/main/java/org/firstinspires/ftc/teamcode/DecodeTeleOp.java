package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@TeleOp(name = "DecodeTeleOp")

public class DecodeTeleOp extends LinearOpMode {
    
    private Hardware hw;
    private Drive drive;
    private TelemetryDashboard dashboard;

    private static final double INTAKE_POWER = 1.0; // Between 0 and 1
    private static final double LOADER_POWER = 0.5; // Between 0 and 1
    private static final double LAUNCHER_IDLE_RPM = 2500;  // maintains spin
    private static final double LAUNCHER_FULL_RPM = 5000;  // full shooting speed
    private static final double TRIGGER_DEADZONE = 0.05;

    @Override
    public void runOpMode() {
        hw = new Hardware();
        hw.setup(hardwareMap);
        
        drive = new Drive();
        drive.setup(hw);
        
        dashboard = new TelemetryDashboard(telemetry, hw);

        waitForStart();
        
        while (opModeIsActive()) {
            drive.update(gamepad1);

            if (gamepad2.right_trigger > TRIGGER_DEADZONE) {
                hw.IntakeServo.setPower(INTAKE_POWER);
            } else {
                hw.IntakeServo.setPower(0);
            }

            double currentLauncherVelocity = hw.Launcher.getVelocity();
            boolean launcherReady = currentLauncherVelocity >= (LAUNCHER_FULL_RPM * 0.95); // Within 95% of FULL_RPM
            if (gamepad2.a && launcherReady) {
                hw.LoaderServo.setPower(LOADER_POWER);
            } else {
                hw.LoaderServo.setPower(0);
            }

            if (gamepad2.right_trigger > TRIGGER_DEADZONE) {
                hw.Launcher.setVelocity(LAUNCHER_FULL_RPM);
            } else {
                hw.Launcher.setVelocity(LAUNCHER_IDLE_RPM);
            }

            dashboard.update(drive);
            idle();
        }
    }
}
