package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.hardware.limelightvision.LLResult;

@TeleOp(name = "TagTest2TeleOp")
public class TagTest2TeleOp extends OpMode {

    private Limelight3A limelight;

    @Override
    public void init() {
        // Map the Limelight from the hardware configuration
        limelight = hardwareMap.get(Limelight3A.class, "Limelight");
        telemetry.addData("Status", "Limelight Initialized");
    }

    @Override
    public void loop() {
        // Get the latest results from the Limelight
        LLResult result = limelight.getLatestResult();

        // Check if the result is valid (i.e., a target is visible)
        if (result.isValid()) {
            // Retrieve the targeting data
            double tx = result.getTx(); // Horizontal Offset from Crosshair to Target (degrees)
            double ty = result.getTy(); // Vertical Offset from Crosshair to Target (degrees)
            double ta = result.getTa(); // Target Area (0-100% of image)
            //int id = result.getID(); // Fiducial (AprilTag) ID

            // Report the data to the Driver Station
            telemetry.addData("Target Found", "Yes");
            //telemetry.addData("Tag ID", id);
            telemetry.addData("Horizontal Offset (tx)", "%.2f degrees", tx);
            telemetry.addData("Vertical Offset (ty)", "%.2f degrees", ty);
            telemetry.addData("Target Area (ta)", "%.2f %%", ta);

        } else {
            telemetry.addData("Target Found", "No");
        }

        telemetry.update();
    }
}