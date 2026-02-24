package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.hardware.VoltageSensor;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;

public class Hardware {
    public static final double TRIGGER_DEADZONE = 0.05;
    public static final double STICK_DEADZONE = 0.05;

    public DigitalChannel tagGreenLed;
    public DigitalChannel tagRedLed;
    public DigitalChannel launcherGreenLed;
    public DigitalChannel launcherRedLed;

    public Limelight3A limelight;

    public IMU imu;
    public IMU.Parameters imuParams;

    public VoltageSensor voltageSensor;

    public AprilTag aprilTag = null;
    public boolean killMotors = false;

    public void setup(HardwareMap hardwareMap) {
//        tagGreenLed = hardwareMap.get(DigitalChannel.class, "TagGreenLed");
//        tagGreenLed.setMode(DigitalChannel.Mode.OUTPUT);
//
//        tagRedLed = hardwareMap.get(DigitalChannel.class, "TagRedLed");
//        tagRedLed.setMode(DigitalChannel.Mode.OUTPUT);
//
//        launcherGreenLed = hardwareMap.get(DigitalChannel.class, "LauncherGreenLed");
//        launcherGreenLed.setMode(DigitalChannel.Mode.OUTPUT);
//
//        launcherRedLed = hardwareMap.get(DigitalChannel.class, "LauncherRedLed");
//        launcherRedLed.setMode(DigitalChannel.Mode.OUTPUT);
//
//        setupLimelight(hardwareMap);

        imu = hardwareMap.get(IMU.class, "imu");
        RevHubOrientationOnRobot orientationOnRobot = new RevHubOrientationOnRobot(
            RevHubOrientationOnRobot.LogoFacingDirection.UP,
            RevHubOrientationOnRobot.UsbFacingDirection.BACKWARD
        );
        imuParams = new IMU.Parameters(orientationOnRobot);
        imu.initialize(imuParams);

        voltageSensor = hardwareMap.voltageSensor.iterator().next();
    }

    public void setupLimelight(HardwareMap hardwareMap) {
        limelight = hardwareMap.get(Limelight3A.class, "Limelight");

        if (limelight != null) {
            aprilTag = new AprilTag(limelight);
        }
    }

    public void toggleMotors() {
        killMotors = ! killMotors;
    }

//    public double getBatteryVoltage() {
//        return voltageSensor.getVoltage();
//    }
}
