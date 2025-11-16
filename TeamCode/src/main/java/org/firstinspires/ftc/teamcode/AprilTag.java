package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.Limelight3A;

import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;

public class AprilTag {
    public Limelight3A limelight;
    public boolean tagSeen = false;
    public double x = Double.NaN;

    public double z = Double.NaN;
    public double zFeet = Double.NaN;

    public AprilTag(Limelight3A limelight) {
        this.limelight = limelight;
    }

    public void start() {
        limelight.setPollRateHz(100);
        limelight.start();
    }

    public void update() {
        LLResult result = limelight.getLatestResult();
        tagSeen = result != null && result.isValid() && !result.getFiducialResults().isEmpty();
        if (tagSeen) {
            Pose3D botPose = result.getBotpose_MT2();
            z = result.getFiducialResults().get(0).getTargetPoseCameraSpace().getPosition().z;
            x = result.getFiducialResults().get(0).getTargetPoseCameraSpace().getPosition().x;
            zFeet = z * 3.28; // There are 3.28 ft in a meter and z is in meters.
        } else {
            z = Double.NaN;
            x = Double.NaN;
        }
    }
}
