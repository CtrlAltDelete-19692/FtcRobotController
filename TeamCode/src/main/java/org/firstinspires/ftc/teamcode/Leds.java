package org.firstinspires.ftc.teamcode;

public class Leds {
    private final Hardware hw;

    public Leds(Hardware hardware) {
        this.hw = hardware;
    }

    public void update(Launcher launcher) {
        if (hw.aprilTag.tagSeen) {
            hw.tagGreenLed.setState(true);
            hw.tagRedLed.setState(false);
        } else {
            hw.tagGreenLed.setState(false);
            hw.tagRedLed.setState(true);
        }

        if (hw.launcher.getVelocity() > launcher.launcherVelocity * 0.94) {
            hw.launcherGreenLed.setState(true);
            hw.launcherRedLed.setState(false);
        } else {
            hw.launcherGreenLed.setState(false);
            hw.launcherRedLed.setState(true);
        }
    }
}
