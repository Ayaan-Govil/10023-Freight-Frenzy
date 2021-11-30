package org.firstinspires.ftc.teamcode.teleops;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.drive.StandardTrackingWheelLocalizer;
import org.firstinspires.ftc.teamcode.hardware.Control;
import org.firstinspires.ftc.teamcode.hardware.Devices;
import org.firstinspires.ftc.teamcode.hardware.Encoders;

@TeleOp

public class RoadRunnerTeleOp extends LinearOpMode {
    private SampleMecanumDrive drive;

    private float finger1XDiff = -100f;
    private float finger1YDiff = -100f;

    private int stage = 0;
    private int firstLevel = 0;
    private int secondLevel = 400;
    private int thirdLevel = 1000;

    private double servoPos = 1.0;
    private ElapsedTime timer = new ElapsedTime();

    public void runOpMode() {
        drive = new SampleMecanumDrive(hardwareMap);

        // Set your initial pose to x: 10, y: 10, facing 90 degrees
        drive.setPoseEstimate(new Pose2d(-60, -35, Math.toRadians(90)));

        Devices.intakeMotor = hardwareMap.get(DcMotor.class, "intake");
        Devices.intakeMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        Devices.conveyorMotor = hardwareMap.get(DcMotor.class, "conveyor");
        Devices.conveyorMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
//        Devices.armLiftMotor = hardwareMap.get(DcMotor.class, "lift");
        Devices.dumpyServo = hardwareMap.get(Servo.class, "dumpy");
        Devices.carouselServo = hardwareMap.get(CRServo.class, "carousel");
        Devices.webcam = hardwareMap.get(WebcamName.class, "Webcam 1");

        telemetry.addLine("Ready to start!");
        telemetry.update();

        waitForStart();

        if (isStopRequested()) return;

        while (opModeIsActive() && !isStopRequested()) {            // Make sure to call drive.update() on *every* loop
            // Increasing loop time by utilizing bulk reads and minimizing writes will increase your odometry accuracy
            drive.setWeightedDrivePower(
                    new Pose2d(
                            -gamepad1.left_stick_y,
                            -gamepad1.left_stick_x,
                            -gamepad1.right_stick_x
                    )
            );
            drive.update();

            // Retrieve your pose
            Pose2d myPose = drive.getPoseEstimate();

            telemetry.addData("x", myPose.getX());
            telemetry.addData("y", myPose.getY());
            telemetry.addData("heading", myPose.getHeading());

//            tankanumDrive(gamepad1.right_stick_y * 0.7, gamepad1.left_stick_y * 0.7, gamepad1.right_stick_x * 0.7);
//
//            if (Encoders.getMotorEnc(Devices.armLiftMotor) < secondLevel - 100) stage = 0;
//            else if (Encoders.getMotorEnc(Devices.armLiftMotor) < thirdLevel - 100) stage = 1;
//            else stage = 2;
//
////        control dumpy 🤤🥴😩
//            if (gamepad1.a && timer.seconds() > 1 && stage != 0) {
//                Devices.dumpyServo.setPosition(servoPos);
//                servoPos = servoPos == 0.6 ? 1.0 : 0.6;
//
//                timer.reset();
//            }
//
//            // intake and conveyor
//            Devices.intakeMotor.setPower(-gamepad1.right_trigger + gamepad1.left_trigger);
//            Devices.conveyorMotor.setPower(gamepad1.right_trigger - gamepad1.left_trigger);

//            if (gamepad1.x) {
//                Devices.carouselServo.setPower(1.0);
//            } else Devices.carouselServo.setPower(0);
//
//            telemetry.addData("lift enc: ", Encoders.getMotorEnc(Devices.armLiftMotor));
//            telemetry.addData("stage: ", stage);
//            telemetry.addData("max: ", Devices.dumpyServo.MAX_POSITION);
//            telemetry.addData("min: ", Devices.dumpyServo.MIN_POSITION);
//            telemetry.addData("current servo pos: ", Devices.dumpyServo.getPosition());
//
//            if (gamepad1.dpad_up && stage < 2) {
//                servoPos = 1.0;
//                Devices.dumpyServo.setPosition(servoPos);
//                Control.motor.linearSlideSetPosition(Devices.armLiftMotor, stage == 0 ? secondLevel : thirdLevel);
//                stage++;
//            } else if (gamepad1.dpad_down && stage > 0) {
//                servoPos = 1.0;
//                Devices.dumpyServo.setPosition(servoPos);
//                Control.motor.linearSlideSetPosition(Devices.armLiftMotor, stage == 2 ? secondLevel : firstLevel);
//                stage--;
//            } else if (gamepad1.dpad_right) {
//                Control.motor.moveMotor(Devices.armLiftMotor, 0.1);
//                if (Encoders.getMotorEnc(Devices.armLiftMotor) >= secondLevel) {
//                    stage = 1;
//                }
//                if (Encoders.getMotorEnc(Devices.armLiftMotor) >= thirdLevel) {
//                    stage = 2;
//                }
//            } else if (gamepad1.dpad_left) {
//                Control.motor.moveMotor(Devices.armLiftMotor, -0.1);
//                if (Encoders.getMotorEnc(Devices.armLiftMotor) <= secondLevel) {
//                    stage = 1;
//                }
//                if (Encoders.getMotorEnc(Devices.armLiftMotor) <= firstLevel) {
//                    stage = 0;
//                }
//            } else Control.motor.moveMotor(Devices.armLiftMotor, 0.0);

            if (gamepad1.touchpad_finger_1) {
                if (finger1XDiff == -100f) finger1XDiff = gamepad1.touchpad_finger_1_x;
                if (finger1YDiff == -100f) finger1YDiff = gamepad1.touchpad_finger_1_y;

                float diffX = gamepad1.touchpad_finger_1_x - finger1XDiff;
                float diffY = gamepad1.touchpad_finger_1_y - finger1YDiff;

                double x = myPose.getX() + (diffX * 10);
                double y = myPose.getY() + (diffY * 10);

                if (x != 0 || y != 0) {
                    drive.followTrajectoryAsync(
                            drive.trajectoryBuilder(myPose)
                                    .lineTo(new Vector2d(x, y))
                                    .build()
                    );
                }

                finger1XDiff = gamepad1.touchpad_finger_1_x;
                finger1YDiff = gamepad1.touchpad_finger_1_y;
            } else {
                finger1XDiff = -100f;
                finger1YDiff = -100f;
            }
        }
    }

    public void tankanumDrive(double rightPwr, double leftPwr, double lateralPwr) {
        double leftFrontPower = Range.clip(leftPwr - lateralPwr, -1.0, 1.0);
        double leftBackPower = Range.clip(leftPwr + lateralPwr, -1.0, 1.0);
        double rightFrontPower = Range.clip(rightPwr - lateralPwr, -1.0, 1.0);
        double rightBackPower = Range.clip(rightPwr + lateralPwr, -1.0, 1.0);

        drive.setMotorPowers(leftFrontPower, leftBackPower, rightBackPower, rightFrontPower);
    }
}