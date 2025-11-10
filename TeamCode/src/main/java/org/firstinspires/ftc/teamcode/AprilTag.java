package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.Limelight3A;

//import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;

public class AprilTag {
    public Limelight3A limelight;

    public AprilTag(Limelight3A limelight) {
        this.limelight = limelight;
    }

    public void start() {
        limelight.setPollRateHz(100);
        limelight.start();
    }

    // Clean up getX and getZ
    public double getX(int tagId) {
        int pipeline = 0; // Blue
        if (tagId == 24) { // TODO: Move these variables into class attr or constants
            pipeline = 1; // Red
        }
        limelight.pipelineSwitch(pipeline);

        LLResult result = limelight.getLatestResult();
        if (result != null) {
            if (result.isValid()) {
                //Pose3D botpose = result.getBotpose();
                return result.getTx();
            }
        }

        return 0;
    }
    public double getZ(int tagId) {
        int pipeline = 0; // Blue
        if (tagId == 24) { // TODO: Move these variables into class attr or constants
            pipeline = 1; // Red
        }
        limelight.pipelineSwitch(pipeline);

        LLResult result = limelight.getLatestResult();
        if (result != null) {
            if (result.isValid()) {
                //Pose3D botpose = result.getBotpose();
                return result.getFiducialResults().get(0).getTargetPoseCameraSpace().getPosition().z;
            }
        }

        return 0;
    }
}
