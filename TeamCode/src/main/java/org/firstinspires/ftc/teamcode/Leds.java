package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.Gamepad;

public class Leds {
    private final Hardware hw;

    public Leds(Hardware hardware) {
        this.hw = hardware;
    }

    public void update(Launcher launcher, Gamepad gamepad) {
        if (! hw.aprilTag.tagSeen) {
            hw.tagGreenLed.setState(true);
            hw.tagRedLed.setState(false);
        } else { // green
            hw.tagGreenLed.setState(false);
            hw.tagRedLed.setState(true);
        }

        if (hw.launcher.getVelocity() <= launcher.launcherVelocity * 0.94 || gamepad.right_trigger <= Hardware.TRIGGER_DEADZONE) { // red
            hw.launcherGreenLed.setState(true);
            hw.launcherRedLed.setState(false);
        } else { // green
            hw.launcherGreenLed.setState(false);
            hw.launcherRedLed.setState(true);
        }
    }
}
