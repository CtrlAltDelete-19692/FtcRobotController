package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@TeleOp(name = "TagTestTeleOp")

public class TagTestTeleOp extends LinearOpMode {

    private Hardware hw;
    private int teamTagId = 20; // TODO: Create function to update team tag Id
    private int pipeline = 0;

    private boolean yPressedLast = false;

    @Override
    public void runOpMode() throws InterruptedException {
        hw = new Hardware();
        hw.setupLimelight(hardwareMap);

        hw.aprilTag.start();
        hw.limelight.pipelineSwitch(pipeline);

        TelemetryDashboard dashboard = new TelemetryDashboard(telemetry, hw);

        waitForStart();
        while (opModeIsActive()) {
            hw.aprilTag.update();

            // Team color change
            boolean yPressed = gamepad1.y;
            if (yPressed && !yPressedLast) {
                toggleTeam();
            }
            yPressedLast = yPressed;

            // Telemetry
            dashboard.update(teamTagId, null, null);
            //idle();
        }
    }

    public void toggleTeam() {
        if (pipeline == 0) {
            pipeline = 1; // Red
            teamTagId = 24;
        } else if (pipeline == 1) {
            pipeline = 0; // Blue
            teamTagId = 20;
        }

        hw.limelight.pipelineSwitch(pipeline);
    }
}
