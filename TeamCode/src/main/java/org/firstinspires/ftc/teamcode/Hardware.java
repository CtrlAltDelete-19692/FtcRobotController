package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.VoltageSensor;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;

public class Hardware {

    public DcMotor RightFrontMotor;
    public DcMotor RightBackMotor;
    public DcMotor LeftFrontMotor;
    public DcMotor LeftBackMotor;
    
    public CRServo IntakeServo;
    public CRServo LoaderServo;
    
    public DcMotorEx Launcher;

    public Limelight3A limelight;

    public IMU imu;
    public IMU.Parameters imuParams;

    public VoltageSensor voltageSensor;

    public void setup(HardwareMap hardwareMap) {
        LeftFrontMotor  = hardwareMap.get(DcMotor.class, "LFM");
        LeftFrontMotor.setDirection(DcMotor.Direction.REVERSE);
        LeftFrontMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        
        LeftBackMotor   = hardwareMap.get(DcMotor.class, "LBM");
        LeftBackMotor.setDirection(DcMotor.Direction.REVERSE);
        LeftBackMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        
        RightFrontMotor = hardwareMap.get(DcMotor.class, "RFM");
        RightFrontMotor.setDirection(DcMotor.Direction.FORWARD);
        RightFrontMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        
        RightBackMotor  = hardwareMap.get(DcMotor.class, "RBM");
        RightBackMotor.setDirection(DcMotor.Direction.FORWARD);
        RightBackMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        
//        IntakeServo = hardwareMap.get(CRServo.class, "IntakeServo");
//        IntakeServo.setDirection(CRServo.Direction.REVERSE); // Test: is this backwards?
//        IntakeServo.setPower(0);
        
        LoaderServo = hardwareMap.get(CRServo.class, "LoaderServo");
        LoaderServo.setDirection(CRServo.Direction.REVERSE);
        LoaderServo.setPower(0);
        
        Launcher = hardwareMap.get(DcMotorEx.class, "LaunchMotor");
        Launcher.setDirection(DcMotor.Direction.FORWARD);
        Launcher.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        Launcher.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        Launcher.setVelocity(0);

        //limelight = hardwareMap.get(Limelight3A.class, "Limelight");

        imu = hardwareMap.get(IMU.class, "imu");
        RevHubOrientationOnRobot orientationOnRobot = new RevHubOrientationOnRobot(
            RevHubOrientationOnRobot.LogoFacingDirection.FORWARD, // TODO & Test: Update according to actual orientation!
            RevHubOrientationOnRobot.UsbFacingDirection.UP
        );
        imuParams = new IMU.Parameters(orientationOnRobot);
        imu.initialize(imuParams);

        voltageSensor = hardwareMap.voltageSensor.iterator().next();
    }

//    public double getBatteryVoltage() {
//        return voltageSensor.getVoltage();
//    }
}
