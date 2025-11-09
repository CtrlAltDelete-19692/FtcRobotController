package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.Limelight3A;

//import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;

public class AprilTag {
    public Limelight3A limelight;

    public AprilTag(Limelight3A limelight) {
        this.limelight = limelight;
    }

    public double getDistance(int tagId) {
        int pipeline = 1; // Blue
        if (tagId == 21) { // TODO: Move these variables into class attr or constants
            pipeline = 0; // Red
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
}
