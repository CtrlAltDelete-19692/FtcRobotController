package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Gamepad;

public class Slides {
    private final Hardware hw;

    private static final double UP_VELOCITY = 2000;
    private static final double DOWN_VELOCITY = -800;
    private static final double MAX_HEIGHT = 6000;
    private static final int MIN_SLIDE_DIFFERENCE = 150; // Minimum slide different before the slowing of one slide occurs
    private static final double MAX_SLIDE_SLOWDOWN = 0.6; // 0 to 1 - Represents the minimum slide speed of the slide that is ahead
    private boolean backPressedLast = false;

    public Slides(Hardware hardware) {
        this.hw = hardware;
    }

    public void update(Gamepad gamepad) {
        int rPos = hw.rightViperSlideMotor.getCurrentPosition();
        int lPos = hw.leftViperSlideMotor.getCurrentPosition();

        int diff = rPos - lPos;

        double leftMultiplier = 1.0;
        double rightMultiplier = 1.0;

        if (Math.abs(diff) > MIN_SLIDE_DIFFERENCE) {
            double adjustment = Math.min(Math.abs(diff) / 1000.0, MAX_SLIDE_SLOWDOWN);

            if (diff > 0) {
                rightMultiplier -= adjustment; // Right is higher -> slow right
            } else {
                leftMultiplier -= adjustment; // Left higher -> slow left
            }
        }

        leftMultiplier  = Math.max(0.0, Math.min(leftMultiplier, 1.0));
        rightMultiplier = Math.max(0.0, Math.min(rightMultiplier, 1.0));

        // TODO: Rewrite to slow the slides down gradually when at the top (or speed up gradually when coming down).
        controlSlide(hw.leftViperSlideMotor, gamepad, leftMultiplier);
        controlSlide(hw.rightViperSlideMotor, gamepad, rightMultiplier);

        setPositionToZero(gamepad);
    }

    private void controlSlide(DcMotorEx motor, Gamepad gamepad, double multiplier) {
        double position = motor.getCurrentPosition();

        if (gamepad.dpad_up && position < MAX_HEIGHT) {
            motor.setVelocity(UP_VELOCITY * multiplier);
        } else if (gamepad.dpad_down) {
            motor.setVelocity(DOWN_VELOCITY * multiplier);
        } else {
            motor.setVelocity(0);
        }
    }

    private void setPositionToZero(Gamepad gamepad) {
        boolean backPressed = gamepad.back || gamepad.share;
        if (backPressed && !backPressedLast) {
            hw.setSlidesToZero();
        }
        backPressedLast = backPressed;
    }
}
