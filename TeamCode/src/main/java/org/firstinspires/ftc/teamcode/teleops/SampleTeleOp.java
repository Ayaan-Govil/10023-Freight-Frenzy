package org.firstinspires.ftc.teamcode.teleops;

import com.qualcomm.hardware.rev.RevBlinkinLedDriver;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
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
    private int firstLevel = 0;
    private int secondLevel = 200;
    private int thirdLevel = 800;

    private double servoPos = 0.0;
    private ElapsedTime timer = new ElapsedTime();

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

//        telemetry.addData("trackpad x: ", gamepad1.touchpad_finger_1_x);
//        telemetry.addData("trackpad y: ", gamepad1.touchpad_finger_1_y);

//        Devices.lightStrip.setPattern(RevBlinkinLedDriver.BlinkinPattern.CP2_SHOT);
//        telemetry.addData("Distance (mm): ", Devices.distanceSensor.getDistance(DistanceUnit.MM));

        // drive using tankanum
//        telemetry.addData("right stick y: ", gamepad1.right_stick_y);
//        Control.drive.tankanumDrive(gamepad1.right_stick_y / 2, gamepad1.left_stick_y / 2, gamepad1.right_stick_x / 2);
//
//        if (Encoders.getMotorEnc(Devices.armLiftMotor) < secondLevel - 100) stage = 0;
//        else if (Encoders.getMotorEnc(Devices.armLiftMotor) < thirdLevel - 100) stage = 1;
//        else stage = 2;
//
////        control dumpy 🤤🥴😩
//        if (gamepad1.a && timer.seconds() > 1) {
//            timer.reset();
//            Devices.dumpyServo.setPosition(servoPos);
//            servoPos = Math.abs(servoPos - 1.0);
//        }

        // intake and conveyor
        Devices.intakeMotor.setPower(-gamepad1.right_trigger);
        Devices.conveyorMotor.setPower(gamepad1.left_trigger);

//        Control.motor.moveMotor(Devices.intakeMotor, gamepad1.right_trigger);
//        Control.motor.moveMotor(Devices.conveyorMotor, -gamepad1.left_trigger);

//        if (gamepad1.x) {
//            Devices.carouselServo.setPower(1.0);
//        } else Devices.carouselServo.setPower(0);

//        telemetry.addData("lift enc: ", Encoders.getMotorEnc(Devices.armLiftMotor));
//        telemetry.addData("stage: ", stage);

//        if (gamepad1.dpad_up && stage < 2) {
//            Control.motor.linearSlideSetPosition(Devices.armLiftMotor, stage == 0 ? secondLevel : thirdLevel);
//            stage++;
//        } else if (gamepad1.dpad_down && stage > 0) {
//            Control.motor.linearSlideSetPosition(Devices.armLiftMotor, stage == 2 ? secondLevel : firstLevel);
//            stage--;
//        } else if (gamepad1.dpad_right) {
//            Control.motor.moveMotor(Devices.armLiftMotor, 0.1);
//            if (Encoders.getMotorEnc(Devices.armLiftMotor) >= secondLevel) {
//                stage = 1;
//            }
//            if (Encoders.getMotorEnc(Devices.armLiftMotor) >= thirdLevel) {
//                stage = 2;
//            }
//        } else if (gamepad1.dpad_left) {
//            Control.motor.moveMotor(Devices.armLiftMotor, -0.1);
//            if (Encoders.getMotorEnc(Devices.armLiftMotor) <= secondLevel) {
//                stage = 1;
//            }
//            if (Encoders.getMotorEnc(Devices.armLiftMotor) <= firstLevel) {
//                stage = 0;
//            }
//        } else Control.motor.moveMotor(Devices.armLiftMotor, 0.0);

//        Devices.armLiftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    }

    @Override
    public void stop() {
        super.stop();
    }
}