package org.firstinspires.ftc.teamcode.hardware;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.rev.RevBlinkinLedDriver;
import com.qualcomm.robotcore.hardware.*;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;

public class Devices {
    // to add a hardware device, initialize the device here and map them in BaseBot
    public static DcMotor leftFrontDriveMotor, rightFrontDriveMotor, leftBackDriveMotor, rightBackDriveMotor, armLiftMotor, carouselMotor;
    public static Servo dumpyServo;
    public static RevBlinkinLedDriver lightStrip;
    public static DistanceSensor distanceSensor;
    public static BNO055IMU imu;
    public static WebcamName webcam;

    // NOTE: deviceName should be the same as the name specified on the configuration
    public static void initDevices(HardwareMap hardwareMap) {
        leftBackDriveMotor = hardwareMap.get(DcMotor.class, "leftBackDriveMotor");
        rightBackDriveMotor = hardwareMap.get(DcMotor.class, "rightBackDriveMotor");
        leftFrontDriveMotor = hardwareMap.get(DcMotor.class, "leftFrontDriveMotor");
        rightFrontDriveMotor = hardwareMap.get(DcMotor.class, "rightFrontDriveMotor");
        Control.drive.configureDriveMotors();

//        carouselMotor = hardwareMap.get(DcMotor.class, "carouselMotor");
//        Devices.carouselMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
//        armLiftMotor = hardwareMap.get(DcMotor.class, "armLiftMotor");
//        armLiftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
//        dumpyServo = hardwareMap.get(Servo.class,"dumpyServo");

//        lightStrip = hardwareMap.get(RevBlinkinLedDriver.class, "lightStrip");
//        distanceSensor = hardwareMap.get(DistanceSensor.class, "distanceSensor");

        imu = hardwareMap.get(BNO055IMU.class, "imu");
//        webcam = hardwareMap.get(WebcamName.class, "Webcam 1");
    }
}
