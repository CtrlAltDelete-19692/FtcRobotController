package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

public class FlowerPuller {
    public static final double DOWN_POSITION = 0.0;
    public static final double UP_POSITION = 1.0;

    private final Servo servo;

    public FlowerPuller(HardwareMap hardwareMap) {
        servo = hardwareMap.tryGet(Servo.class, "FlowerPuller");
    }

    public void update(Gamepad gamepad) {
        if (gamepad.dpad_up) {
            setUp();
        } else if (gamepad.dpad_down) {
            setDown();
        }
    }

    public void setUp() {
        if (servo != null) servo.setPosition(UP_POSITION);
    }

    public void setDown() {
        if (servo != null) servo.setPosition(DOWN_POSITION);
    }

    public boolean isConfigured() {
        return servo != null;
    }
}
