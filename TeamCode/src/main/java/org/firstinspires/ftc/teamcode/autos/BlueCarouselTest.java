package org.firstinspires.ftc.teamcode.autos;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.bosch.JustLoggingAccelerationIntegrator;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.Acceleration;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;

import org.firstinspires.ftc.robotcore.external.tfod.Recognition;
import org.firstinspires.ftc.teamcode.BaseRobot;
import org.firstinspires.ftc.teamcode.hardware.*;

import java.util.List;

@Autonomous

// basic autonomous that moves forward 10 inches, waits 5 seconds, turns around, then drives back

public class BlueCarouselTest extends LinearOpMode {

//    @Override
//    public void init() {
//        super.init();
//    }
//
//    @Override
//    public void start() {
//        super.start();
//        Control.sensor.initGyro();
//        Control.auto.moveWithEncoder(10, 0.5);
//        Control.auto.turnWithGyro(90, 0.5);
//    }
//
//    @Override
//    public void loop() {
//        super.loop();
//    }

    public void runOpMode() {
        Devices.initDevices(hardwareMap);
//        Control.sensor.initGyro();

        telemetry.addData(">", "Press Play to start op mode");
        telemetry.update();

        waitForStart();

        Control.motor.linearSlideSetPosition(Devices.armLiftMotor, 800);
    }

    public void carouselTime(int milliseconds, double speed) {
        ElapsedTime timer = new ElapsedTime();

        Devices.carouselServo.setPower(speed);

        while (timer.milliseconds() < milliseconds) {
        }

        Devices.carouselServo.setPower(0.0);
    }

    public void mecanumTime(int milliseconds, double speed) {
        ElapsedTime timer = new ElapsedTime();

        Devices.leftFrontDriveMotor.setPower(speed);
        Devices.leftBackDriveMotor.setPower(-speed);
        Devices.rightFrontDriveMotor.setPower(speed);
        Devices.rightBackDriveMotor.setPower(-speed);

        while (timer.milliseconds() < milliseconds) {

        }

        Devices.leftFrontDriveMotor.setPower(0);
        Devices.leftBackDriveMotor.setPower(0);
        Devices.rightFrontDriveMotor.setPower(0);
        Devices.rightBackDriveMotor.setPower(0);
    }

    public void turnTime(int milliseconds, double speed) {
        ElapsedTime timer = new ElapsedTime();

        Devices.leftFrontDriveMotor.setPower(-speed);
        Devices.leftBackDriveMotor.setPower(-speed);
        Devices.rightFrontDriveMotor.setPower(speed);
        Devices.rightBackDriveMotor.setPower(speed);

        while (timer.milliseconds() < milliseconds) {

        }

        Devices.leftFrontDriveMotor.setPower(0);
        Devices.leftBackDriveMotor.setPower(0);
        Devices.rightFrontDriveMotor.setPower(0);
        Devices.rightBackDriveMotor.setPower(0);
    }

    public void moveTime(int milliseconds, double speed) {
        ElapsedTime timer = new ElapsedTime();

        Devices.leftFrontDriveMotor.setPower(speed);
        Devices.leftBackDriveMotor.setPower(speed);
        Devices.rightFrontDriveMotor.setPower(speed);
        Devices.rightBackDriveMotor.setPower(speed);

        while (timer.milliseconds() < milliseconds) {

        }

        Devices.leftFrontDriveMotor.setPower(0);
        Devices.leftBackDriveMotor.setPower(0);
        Devices.rightFrontDriveMotor.setPower(0);
        Devices.rightBackDriveMotor.setPower(0);
    }
}