package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.Limelight3A;

public class AprilTag {
    public Limelight3A limelight;

    public AprilTag(Limelight3A limelight) {
        this.limelight = limelight;
    }

    public void start() {
        limelight.setPollRateHz(100);
        limelight.start();
    }

    public void setPipeline(int tagId) {
        int pipeline = -1;
        if (tagId == 20) {
            pipeline = 0;
        } else if (tagId == 24) {
            pipeline = 1;
        } else if (tagId == 21 || tagId == 22 || tagId == 23) {
            pipeline = 2;
        }

        if (pipeline != -1) {
            limelight.pipelineSwitch(pipeline);
        }
    }

    // Clean up getX and getZ
    public double getX() {
        LLResult result = limelight.getLatestResult();
        if (result != null) {
            if (result.isValid()) {
                //return result.getTx();
                return result.getFiducialResults().get(0).getTargetPoseCameraSpace().getPosition().x;
            }
        }

        return 0;
    }
    public double getZ() {
        LLResult result = limelight.getLatestResult();
        if (result != null) {
            if (result.isValid()) {
                return result.getFiducialResults().get(0).getTargetPoseCameraSpace().getPosition().z;
            }
        }

        return 0;
    }
}
