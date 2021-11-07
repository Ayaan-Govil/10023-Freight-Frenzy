package org.firstinspires.ftc.teamcode.hardware;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.rev.RevBlinkinLedDriver;
import com.qualcomm.robotcore.hardware.*;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;

public class Devices {
    // to add a hardware device, initialize the device here and map them in BaseBot
    public static DcMotor leftFrontDriveMotor, rightFrontDriveMotor, leftBackDriveMotor, rightBackDriveMotor, armLiftMotor, conveyorMotor, intakeMotor;
    public static Servo dumpyServo;
    public static CRServo carouselServo;
    public static RevBlinkinLedDriver lightStrip;
    public static DistanceSensor distanceSensor;
    public static BNO055IMU imu;
    public static WebcamName webcam;

    // NOTE: deviceName should be the same as the name specified on the configuration
    public static void initDevices(HardwareMap hardwareMap) {
//        leftBackDriveMotor = hardwareMap.get(DcMotorEx.class, "leftBackDriveMotor");
//        rightBackDriveMotor = hardwareMap.get(DcMotorEx.class, "rightBackDriveMotor");
//        leftFrontDriveMotor = hardwareMap.get(DcMotorEx.class, "leftFrontDriveMotor");
//        rightFrontDriveMotor = hardwareMap.get(DcMotorEx.class, "rightFrontDriveMotor");
//        Control.drive.configureDriveMotors();

        intakeMotor = hardwareMap.get(DcMotor.class, "intakeMotor");
        intakeMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        conveyorMotor = hardwareMap.get(DcMotor.class, "conveyorMotor");
        conveyorMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        armLiftMotor = hardwareMap.get(DcMotor.class, "armLiftMotor");
        dumpyServo = hardwareMap.get(Servo.class, "dumpyServo");
        carouselServo = hardwareMap.get(CRServo.class, "carouselServo");
//        lightStrip = hardwareMap.get(RevBlinkinLedDriver.class, "lightStrip");
//        distanceSensor = hardwareMap.get(DistanceSensor.class, "distanceSensor");

        imu = hardwareMap.get(BNO055IMU.class, "imu");
//        webcam = hardwareMap.get(WebcamName.class, "Webcam 1");
    }
}
