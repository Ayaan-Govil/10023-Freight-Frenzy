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
        leftBackDriveMotor = hardwareMap.get(DcMotorEx.class, "leftBack");
        rightBackDriveMotor = hardwareMap.get(DcMotorEx.class, "rightBack");
        leftFrontDriveMotor = hardwareMap.get(DcMotorEx.class, "leftFront");
        rightFrontDriveMotor = hardwareMap.get(DcMotorEx.class, "rightFront");
        Control.drive.configureDriveMotors();

        intakeMotor = hardwareMap.get(DcMotor.class, "intake");
        intakeMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        conveyorMotor = hardwareMap.get(DcMotor.class, "conveyor");
        conveyorMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        armLiftMotor = hardwareMap.get(DcMotor.class, "lift");
        armLiftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        dumpyServo = hardwareMap.get(Servo.class, "dumpy");
        carouselServo = hardwareMap.get(CRServo.class, "carousel");
//        lightStrip = hardwareMap.get(RevBlinkinLedDriver.class, "lightStrip");
//        distanceSensor = hardwareMap.get(DistanceSensor.class, "distanceSensor");

        imu = hardwareMap.get(BNO055IMU.class, "imu");
        webcam = hardwareMap.get(WebcamName.class, "Webcam 1");
    }
}
