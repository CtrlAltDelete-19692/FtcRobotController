package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.LED;

public class Leds {
    private final Hardware hw;

    public Leds(Hardware hardware) {
        this.hw = hardware;
    }

    public void update(Launcher launcher) {
        if (hw.aprilTag.tagSeen) {
            setTagState("green");
        } else {
            setTagState("red");
        }

        if (launcher.readyToLaunch) {
            setLauncherState("green");
        } else {
            setLauncherState("red");
        }
    }

    private void setTagState(String state) {
        if (state.equals("green")) {
            hw.tagGreenLed.setState(false);
            hw.tagRedLed.setState(true);
        } else if (state.equals("red")) {
            hw.tagGreenLed.setState(true);
            hw.tagRedLed.setState(false);
        }
    }

    private void setLauncherState(String state) {
        if (state.equals("green")) {
            hw.launcherGreenLed.setState(false);
            hw.launcherRedLed.setState(true);
        } else if (state.equals("red")) {
            hw.launcherGreenLed.setState(true);
            hw.launcherRedLed.setState(false);
        }
    }
}
