package org.firstinspires.ftc.teamcode.teleops;

import com.qualcomm.hardware.rev.RevBlinkinLedDriver;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
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

        Devices.lightStrip.setPattern(RevBlinkinLedDriver.BlinkinPattern.CP2_SHOT);
//        telemetry.addData("Distance (mm): ", Devices.distanceSensor.getDistance(DistanceUnit.MM));

        // drive using tankanum
//        Control.tankanumDrive(gamepad1.right_stick_y, gamepad1.left_stick_y, gamepad1.right_stick_x);

        if (gamepad1.a) {
            Devices.armAdjustServo.setPosition(1.0);
        } else if (gamepad1.b) {
            Devices.armAdjustServo.setPosition(0.0);
        }

        // control armLiftMotor
//        int stageSpaces = 500;
//        double speed = 1.0;
//        telemetry.addData("stage: ", stage);
//
//        if (gamepad1.dpad_up && stage < 2) {
//            int targetEnc = (stage + 1) * stageSpaces;
//            while (Encoders.getMotorEnc((Devices.armLiftMotor)) < targetEnc) {
//                Control.moveMotor(Devices.armLiftMotor, speed);
//            }
//            Control.moveMotor(Devices.armLiftMotor, 0.0);
//            stage++;
//        } else if (gamepad1.dpad_down && stage > 0) {
//            int targetEnc = (stage - 1) * stageSpaces;
//            if (targetEnc == 0) targetEnc = 50;
//            while (Encoders.getMotorEnc((Devices.armLiftMotor)) > targetEnc) {
//                Control.moveMotor(Devices.armLiftMotor, -speed);
//            }
//            Control.moveMotor(Devices.armLiftMotor, 0.0);
//            stage--;
//        } else if (gamepad1.b) {
//            Control.moveMotor(Devices.armLiftMotor, 0.1);
//            if (Encoders.getMotorEnc(Devices.armLiftMotor) >= stageSpaces) {
//                stage = 1;
//            }
//            if (Encoders.getMotorEnc(Devices.armLiftMotor) >= stageSpaces * 2) {
//                stage = 2;
//            }
//        } else if (gamepad1.a) {
//            Control.moveMotor(Devices.armLiftMotor, -0.1);
//            if (Encoders.getMotorEnc(Devices.armLiftMotor) <= stageSpaces) {
//                stage = 1;
//            }
//            if (Encoders.getMotorEnc(Devices.armLiftMotor) <= 0) {
//                stage = 0;
//            }
//        } else Control.moveMotor(Devices.armLiftMotor, 0.0);

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