package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

public class CtrlAltDelOpMode extends LinearOpMode {
    public static final double TRIGGER_DEADZONE = 0.05;
    public static final double STICK_DEADZONE = 0.05;
    public static boolean killMotors = false;

    public void runOpMode() throws InterruptedException {
    }

    public void toggleMotors() {
        killMotors = ! killMotors;
    }
}
