package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Gamepad;

public class Slides {
    private final Hardware hw;

    private static final double UP_VELOCITY = 2000;
    private static final double DOWN_VELOCITY = -800;
    private static final double MAX_HEIGHT = 6000;
    private static final double MIN_HEIGHT = 0;
    private double leftMultiplier = 1;
    private double rightMultiplier = 1;

    public Slides(Hardware hardware) {
        this.hw = hardware;
    }

    public void update(Gamepad gamepad) {

        int rPos = hw.rightViperSlideMotor.getCurrentPosition();
        int lPos = hw.leftViperSlideMotor.getCurrentPosition();

        int diff = rPos - lPos;
        if (diff > 150) { // Right too high
            //leftMultiplier = 0.8;
            //rightMultiplier = 1;
        } else if (diff < 150) { // Left too high
            //leftMultiplier = 1;
            //rightMultiplier = 0.8;
        } else {
            leftMultiplier = 1;
            rightMultiplier = 1;
        }

        // TODO: Rewrite to slow the slides down gradually when at the top (or speed up gradually when coming down).
        // TODO: Deal with starting the bot while slides are not in home position (shouldn't happen, but would be nice to account for).
        controlSlide(hw.leftViperSlideMotor, gamepad, leftMultiplier);
        controlSlide(hw.rightViperSlideMotor, gamepad, rightMultiplier);
    }

    private void controlSlide(DcMotorEx motor, Gamepad gamepad, double multiplier) {
        double position = motor.getCurrentPosition();

        if (gamepad.dpad_up && position < MAX_HEIGHT) {
            motor.setVelocity(UP_VELOCITY);
        } else if (gamepad.dpad_down && position > MIN_HEIGHT) {
            motor.setVelocity(DOWN_VELOCITY);
        } else {
            motor.setVelocity(0);
        }
    }
}
