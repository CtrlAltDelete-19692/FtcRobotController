package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.VoltageSensor;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;

public class Hardware {

    public DcMotor rightFrontMotor;
    public DcMotor rightBackMotor;
    public DcMotor leftFrontMotor;
    public DcMotor leftBackMotor;
    
    public CRServo intake;
    public CRServo loader;
    
    public DcMotorEx launcher;

    public DcMotorEx   leftViperSlideMotor;
    public DcMotorEx   rightViperSlideMotor;

    public Limelight3A limelight;

    public IMU imu;
    public IMU.Parameters imuParams;

    public VoltageSensor voltageSensor;

    public void setup(HardwareMap hardwareMap) {
        leftFrontMotor = hardwareMap.get(DcMotor.class, "LFM");
        leftFrontMotor.setDirection(DcMotor.Direction.REVERSE);
        leftFrontMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        
        leftBackMotor  = hardwareMap.get(DcMotor.class, "LBM");
        leftBackMotor.setDirection(DcMotor.Direction.REVERSE);
        leftBackMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        
        rightFrontMotor = hardwareMap.get(DcMotor.class, "RFM");
        rightFrontMotor.setDirection(DcMotor.Direction.FORWARD);
        rightFrontMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        
        rightBackMotor = hardwareMap.get(DcMotor.class, "RBM");
        rightBackMotor.setDirection(DcMotor.Direction.FORWARD);
        rightBackMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

//        intake = hardwareMap.get(CRServo.class, "Intake");
//        intake.setDirection(CRServo.Direction.REVERSE); // Might be backwards, it can't be tested as it no longer exists on the physical bot
//        intake.setPower(0);

        loader = hardwareMap.get(CRServo.class, "Loader");
        if (loader != null) {
            loader.setDirection(CRServo.Direction.REVERSE);
            loader.setPower(0);
        }
        
        launcher = hardwareMap.get(DcMotorEx.class, "Launcher");
        launcher.setDirection(DcMotor.Direction.FORWARD);
        launcher.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        launcher.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        launcher.setVelocity(0);

        leftViperSlideMotor = hardwareMap.get(DcMotorEx.class, "LeftViperMotor");
        leftViperSlideMotor.setDirection(DcMotor.Direction.FORWARD);
        leftViperSlideMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        leftViperSlideMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftViperSlideMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        rightViperSlideMotor = hardwareMap.get(DcMotorEx.class, "RightViperMotor");
        rightViperSlideMotor.setDirection(DcMotor.Direction.FORWARD);
        rightViperSlideMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightViperSlideMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightViperSlideMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        limelight = hardwareMap.get(Limelight3A.class, "Limelight");

        imu = hardwareMap.get(IMU.class, "imu");
        RevHubOrientationOnRobot orientationOnRobot = new RevHubOrientationOnRobot(
            RevHubOrientationOnRobot.LogoFacingDirection.FORWARD,
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
