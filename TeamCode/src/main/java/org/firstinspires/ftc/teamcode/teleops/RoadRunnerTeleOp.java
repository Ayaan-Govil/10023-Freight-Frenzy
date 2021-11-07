package org.firstinspires.ftc.teamcode.teleops;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.drive.StandardTrackingWheelLocalizer;
import org.firstinspires.ftc.teamcode.hardware.Control;
import org.firstinspires.ftc.teamcode.hardware.Devices;

@TeleOp

public class RoadRunnerTeleOp extends LinearOpMode {
    private SampleMecanumDrive drive;

    private float finger1XDiff = -100f;
    private float finger1YDiff = -100f;

    private float finger2XDiff = -100f;

    public void runOpMode() {
        drive = new SampleMecanumDrive(hardwareMap);

        drive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        // Set your initial pose to x: 10, y: 10, facing 90 degrees
        drive.setPoseEstimate(new Pose2d(10, 10, Math.toRadians(90)));

        telemetry.addData(">", "Press Play to start op mode");
        telemetry.update();

        waitForStart();

        if (isStopRequested()) return;

        while (opModeIsActive() && !isStopRequested()) {
            drive.setWeightedDrivePower(
                    new Pose2d(
                            -gamepad1.left_stick_y,
                            -gamepad1.left_stick_x,
                            -gamepad1.right_stick_x
                    )
            );

//            drive.trajectoryBuilder()

            // Make sure to call drive.update() on *every* loop
            // Increasing loop time by utilizing bulk reads and minimizing writes will increase your odometry accuracy
            drive.update();

            // Retrieve your pose
            Pose2d myPose = drive.getPoseEstimate();

            telemetry.addData("x", myPose.getX());
            telemetry.addData("y", myPose.getY());
            telemetry.addData("heading", myPose.getHeading());
            telemetry.update();

            // Insert whatever teleop code you're using
//            this.tankanumDrive(gamepad1.right_stick_y * 0.6, gamepad1.left_stick_y * 0.6, gamepad1.right_stick_x * 0.6);

            if (gamepad1.touchpad_finger_1) {
                if (finger1XDiff == -100f) finger1XDiff = gamepad1.touchpad_finger_1_x;
                if (finger1YDiff == -100f) finger1YDiff = gamepad1.touchpad_finger_1_y;

                float diffX = gamepad1.touchpad_finger_1_x - finger1XDiff;
                float diffY = gamepad1.touchpad_finger_1_y - finger1YDiff;

                drive.followTrajectoryAsync(
                        drive.trajectoryBuilder(myPose)
                                .lineTo(new Vector2d(myPose.getX() + (diffX * 10), myPose.getY() + (diffY * 10)))
                                .build()
                );

                finger1XDiff = gamepad1.touchpad_finger_1_x;
                finger1YDiff = gamepad1.touchpad_finger_1_y;
            } else {
                finger1XDiff = -100f;
                finger1YDiff = -100f;
            }

//            if (gamepad1.touchpad_finger_2) {
//                if (finger2XDiff == -100f) finger2XDiff = gamepad1.touchpad_finger_2_x;
//
//                float diffX = gamepad1.touchpad_finger_2_x - finger2XDiff;
//
//                drive.turnAsync(Math.toRadians(diffX));
//
//                finger2XDiff = gamepad1.touchpad_finger_2_x;
//            } else {
//                finger2XDiff = -100f;
//            }
        }
    }

//    public void tankanumDrive(double rightPwr, double leftPwr, double lateralPwr) {
//        double leftFrontPower = Range.clip(leftPwr - lateralPwr, -1.0, 1.0);
//        double leftBackPower = Range.clip(leftPwr + lateralPwr, -1.0, 1.0);
//        double rightFrontPower = Range.clip(rightPwr - lateralPwr, -1.0, 1.0);
//        double rightBackPower = Range.clip(rightPwr + lateralPwr, -1.0, 1.0);
//
//        drive.setMotorPowers(leftFrontPower, leftBackPower, rightBackPower, rightFrontPower);
//    }
}