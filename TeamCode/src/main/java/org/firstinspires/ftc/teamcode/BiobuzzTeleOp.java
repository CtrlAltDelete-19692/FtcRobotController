package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

@TeleOp(name = "BiobuzzTeleOp")

public class BiobuzzTeleOp extends CtrlAltDelOpMode {
    public Drive drive;
    private RotatingLauncher pollenLauncher;
    private RotatingLauncher nectarLauncher;
    private Intake intake;
    private int teamTagId = 20;

    @Override
    public void runOpMode() {
        // Initialize Subsystems
        drive = new Drive(hardwareMap, "Biobuzz", false);
        drive.setResetYawOnRightBumper(false);
        intake = new Intake(hardwareMap, DcMotor.Direction.REVERSE);
        pollenLauncher = new RotatingLauncher(hardwareMap, "Pollen", "PollenFlywheel",
                "PollenLoader", "PollenRotatingServo", null, null);
        nectarLauncher = new RotatingLauncher(hardwareMap, "Nectar", "NectarFlywheel",
                "NectarLoader", "NectarRotatingServo", null, null);

        TelemetryDashboard dashboard = new TelemetryDashboard(telemetry);

        while (!isStarted()) {
            //if (gamepad1.x) changeTeam(true);
            //if (gamepad1.b) changeTeam(false);

            // Telemetry Setup
            dashboard.addStatus(teamTagId, drive.getDriveMode(), killMotors);
            pollenLauncher.addTelemetry(telemetry);
            nectarLauncher.addTelemetry(telemetry);
            intake.addTelemetry(telemetry);
            drive.addTelemetry(telemetry);
            dashboard.update();
        }

        waitForStart();

        drive.imu.resetYaw();

        while (opModeIsActive()) {
            // Run update() for all subsystems
            drive.update(gamepad1, gamepad2, teamTagId);
            intake.update(gamepad1, gamepad2);
            pollenLauncher.update(gamepad1);
            nectarLauncher.update(gamepad1);

            // Telemetry
            dashboard.addStatus(teamTagId, drive.getDriveMode(), killMotors);
            pollenLauncher.addTelemetry(telemetry);
            nectarLauncher.addTelemetry(telemetry);
            intake.addTelemetry(telemetry);
            drive.addTelemetry(telemetry);
            dashboard.update();
        }
    }
}
