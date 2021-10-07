package org.firstinspires.ftc.teamcode.autos;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.bosch.JustLoggingAccelerationIntegrator;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

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

public class Calibration extends LinearOpMode {

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
        Control.sensor.initGyro();

        telemetry.addData(">", "Press Play to start op mode");
        telemetry.update();

        waitForStart();

        Control.auto.strafeToPosition(20, 0.5);
//        Control.auto.moveWithEncoder(13, 0.5);
//        double inches = 20;
//        double speed = 1;
//        double meccyBias = 0.9;
//        int move = (int) (Math.round(inches * ConstantVariables.COUNTS_PER_INCH * meccyBias));
//
//        Devices.leftFrontDriveMotor.setTargetPosition(Devices.leftFrontDriveMotor.getCurrentPosition() - move);
//        Devices.leftBackDriveMotor.setTargetPosition(Devices.leftBackDriveMotor.getCurrentPosition() + move);
//        Devices.rightFrontDriveMotor.setTargetPosition(Devices.rightFrontDriveMotor.getCurrentPosition() + move);
//        Devices.rightBackDriveMotor.setTargetPosition(Devices.rightBackDriveMotor.getCurrentPosition() - move);
//
//        Encoders.driveRunToPosition();
//
////        Devices.rightFrontDriveMotor.setDirection(DcMotor.Direction.FORWARD);
////        Devices.rightBackDriveMotor.setDirection(DcMotor.Direction.FORWARD);
//
//        Devices.leftFrontDriveMotor.setPower(-speed);
//        Devices.leftBackDriveMotor.setPower(speed);
//        Devices.rightFrontDriveMotor.setPower(speed);
//        Devices.rightBackDriveMotor.setPower(speed);
//
//        while (Devices.leftFrontDriveMotor.isBusy() && Devices.leftBackDriveMotor.isBusy() && Devices.rightFrontDriveMotor.isBusy() && Devices.rightBackDriveMotor.isBusy()) {
//
//        }
//
//        Control.drive.stopPower();
    }
}
