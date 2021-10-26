package org.firstinspires.ftc.teamcode.teleops;

import com.qualcomm.hardware.rev.RevBlinkinLedDriver;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.Const;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.BaseRobot;
import org.firstinspires.ftc.teamcode.hardware.Control;
import org.firstinspires.ftc.teamcode.hardware.Devices;
import org.firstinspires.ftc.teamcode.hardware.Encoders;

import com.qualcomm.robotcore.hardware.I2cAddr;
import com.qualcomm.robotcore.hardware.I2cDeviceSynch;

@TeleOp

public class SampleTeleOp extends BaseRobot {
    private int stage = 0;
//    private double power = 1.0;


    @Override
    public void init() {
        super.init();
    }

    @Override
    public void start() {
        super.start();
    }

    @Override
    public void loop() {
        super.loop();

//        Devices.lightStrip.setPattern(RevBlinkinLedDriver.BlinkinPattern.CP2_SHOT);
//        telemetry.addData("Distance (mm): ", Devices.distanceSensor.getDistance(DistanceUnit.MM));

        // drive using tankanum
        telemetry.addData("right stick y: ", gamepad1.right_stick_y);
        Control.drive.tankanumDrive(gamepad1.right_stick_y/2, gamepad1.left_stick_y/2, gamepad1.right_stick_x/2);

//        if (Encoders.getMotorEnc(Devices.armLiftMotor) < 400) stage = 0;
//        else if (Encoders.getMotorEnc(Devices.armLiftMotor) < 900) stage = 1;
//        else stage = 2;
//
//        // control dumpy 🤤🥴😩
//        if (gamepad1.dpad_right) {
//            Devices.dumpyServo.setPosition(1.0);
//        } else if (gamepad1.dpad_left) {
//            Devices.dumpyServo.setPosition(0.0);
//        }
//
//        telemetry.addData("lift enc: ", Encoders.getMotorEnc(Devices.armLiftMotor));
//        telemetry.addData("stage: ", stage);
//
//        Devices.armLiftMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
//
//        if (gamepad1.dpad_up && stage < 2) {
//            stage++;
//            Control.motor.linearSlideSetPosition(Devices.armLiftMotor, stage);
//        } else if (gamepad1.dpad_down && stage > 0) {
//            stage--;
//            Control.motor.linearSlideSetPosition(Devices.armLiftMotor, stage);
//        } else if (gamepad1.b) {
//            Control.motor.moveMotor(Devices.armLiftMotor, 0.1);
//            if (Encoders.getMotorEnc(Devices.armLiftMotor) >= 500) {
//                stage = 1;
//            }
//            if (Encoders.getMotorEnc(Devices.armLiftMotor) >= 1000) {
//                stage = 2;
//            }
//        } else if (gamepad1.a) {
//            Control.motor.moveMotor(Devices.armLiftMotor, -0.1);
//            if (Encoders.getMotorEnc(Devices.armLiftMotor) <= 500) {
//                stage = 1;
//            }
//            if (Encoders.getMotorEnc(Devices.armLiftMotor) <= 0) {
//                stage = 0;
//            }
//        } else Control.motor.moveMotor(Devices.armLiftMotor, 0.0);

        // control armClampServo
//        if (gamepad1.left_trigger > 0.0 || gamepad1.left_bumper) {
//            Control.setServoPosition(Devices.armClampServo, 0.0);
//        } else if (gamepad1.right_trigger > 0.0 || gamepad1.right_bumper) {
//            Control.setServoPosition(Devices.armClampServo, 1.0);
//        }
    }

    @Override
    public void stop() {
        super.stop();
    }
}