package org.firstinspires.ftc.teamcode.teleops;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.drive.StandardTrackingWheelLocalizer;
import org.firstinspires.ftc.teamcode.hardware.Control;
import org.firstinspires.ftc.teamcode.hardware.Devices;

@TeleOp

public class RoadRunnerTeleOp extends LinearOpMode {
    private SampleMecanumDrive myLocalizer;

    public void runOpMode() {
        // Insert whatever initialization your own code does
//        Devices.initDevices(hardwareMap);

        // This is assuming you're using StandardTrackingWheelLocalizer.java
        // Switch this class to something else (Like TwoWheeTrackingLocalizer.java) if your configuration is different
        myLocalizer = new SampleMecanumDrive(hardwareMap);

        // Set your initial pose to x: 10, y: 10, facing 90 degrees
        myLocalizer.setPoseEstimate(new Pose2d(10, 10, Math.toRadians(90)));

        telemetry.addData(">", "Press Play to start op mode");
        telemetry.update();
        waitForStart();

        while (opModeIsActive()) {
            // Make sure to call myLocalizer.update() on *every* loop
            // Increasing loop time by utilizing bulk reads and minimizing writes will increase your odometry accuracy
            myLocalizer.update();

            // Retrieve your pose
            Pose2d myPose = myLocalizer.getPoseEstimate();

            telemetry.addData("x", myPose.getX());
            telemetry.addData("y", myPose.getY());
            telemetry.addData("heading", myPose.getHeading());
            telemetry.addData("right stick y: ", gamepad1.right_stick_y);
            telemetry.addData("right stick x: ", gamepad1.right_stick_x);
            telemetry.update();

            // Insert whatever teleop code you're using
            this.tankanumDrive(gamepad1.right_stick_y * 0.6, gamepad1.left_stick_y * 0.6, gamepad1.right_stick_x * 0.6);
        }
    }

    public void tankanumDrive(double rightPwr, double leftPwr, double lateralPwr) {
        double leftFrontPower = Range.clip(leftPwr - lateralPwr, -1.0, 1.0);
        double leftBackPower = Range.clip(leftPwr + lateralPwr, -1.0, 1.0);
        double rightFrontPower = Range.clip(rightPwr - lateralPwr, -1.0, 1.0);
        double rightBackPower = Range.clip(rightPwr + lateralPwr, -1.0, 1.0);

        myLocalizer.setMotorPowers(leftFrontPower, leftBackPower, rightBackPower, rightFrontPower);
    }
}