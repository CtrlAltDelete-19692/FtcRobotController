package org.firstinspires.ftc.teamcode;

public class BallPurge {
    private final Intake intake;
    private final RotatingLauncher launcher;

    public BallPurge(Intake intake, RotatingLauncher launcher) {
        this.intake = intake;
        this.launcher = launcher;
    }

    public void setRunning(boolean running) {
        launcher.setPurgeRunning(running);
        intake.setPurgeRunning(running);
    }

    public boolean isRunning() {
        return intake.isPurgeRunning();
    }
}
