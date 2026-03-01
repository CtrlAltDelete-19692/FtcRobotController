package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.LED;

public class Leds {
    public DigitalChannel tagGreenLed;
    public DigitalChannel tagRedLed;
    public DigitalChannel launcherGreenLed;
    public DigitalChannel launcherRedLed;
    public AprilTag aprilTag;
    public Launcher launcher;

    public Leds(HardwareMap hardwareMap, AprilTag aprilTag, Launcher launcher) {
        tagGreenLed = hardwareMap.get(DigitalChannel.class, "TagGreenLed");
        tagGreenLed.setMode(DigitalChannel.Mode.OUTPUT);

        tagRedLed = hardwareMap.get(DigitalChannel.class, "TagRedLed");
        tagRedLed.setMode(DigitalChannel.Mode.OUTPUT);

        launcherGreenLed = hardwareMap.get(DigitalChannel.class, "LauncherGreenLed");
        launcherGreenLed.setMode(DigitalChannel.Mode.OUTPUT);

        launcherRedLed = hardwareMap.get(DigitalChannel.class, "LauncherRedLed");
        launcherRedLed.setMode(DigitalChannel.Mode.OUTPUT);

        this.aprilTag = aprilTag;
        this.launcher = launcher;
    }

    public void update() {
        if (aprilTag.tagSeen) {
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
            tagGreenLed.setState(false);
            tagRedLed.setState(true);
        } else if (state.equals("red")) {
            tagGreenLed.setState(true);
            tagRedLed.setState(false);
        }
    }

    private void setLauncherState(String state) {
        if (state.equals("green")) {
            launcherGreenLed.setState(false);
            launcherRedLed.setState(true);
        } else if (state.equals("red")) {
            launcherGreenLed.setState(true);
            launcherRedLed.setState(false);
        }
    }
}
