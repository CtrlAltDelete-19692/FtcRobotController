package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.limelightvision.Limelight3A;

import org.firstinspires.ftc.robotcore.external.Telemetry;

/** Keeps alliance selection and Limelight pipeline selection together. */
public class BiobuzzTeamSelection {
    private final Limelight3A limelight;
    private final Telemetry telemetry;
    private int selectedPipeline = -1;
    private boolean blue = true;

    public BiobuzzTeamSelection(Limelight3A limelight, Telemetry telemetry) {
        this.limelight = limelight;
        this.telemetry = telemetry;
    }

    public void select(boolean blue) {
        int pipeline = blue ? 2 : 1;
        if (selectedPipeline == pipeline) return;
        this.blue = blue;
        selectedPipeline = pipeline;
        telemetry.speak(blue ? "Blue team" : "Red team");
        if (limelight != null) limelight.pipelineSwitch(pipeline);
    }

    public boolean isBlue() {
        return blue;
    }
}
