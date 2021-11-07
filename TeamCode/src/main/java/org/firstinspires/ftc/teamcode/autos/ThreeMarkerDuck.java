package org.firstinspires.ftc.teamcode.autos;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
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
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;

import org.firstinspires.ftc.robotcore.external.tfod.Recognition;
import org.firstinspires.ftc.teamcode.BaseRobot;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.hardware.*;

import java.util.List;

//@Autonomous
// basic autonomous that moves forward 10 inches, waits 5 seconds, turns around, then drives back

public class ThreeMarkerDuck extends LinearOpMode {
    public int coeff = 1;

    public void runOpMode() {
//        Devices.initDevices(hardwareMap);
        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);
        Pose2d startPos = new Pose2d(coeff * -35, coeff * -61, Math.toRadians(90));
        drive.setPoseEstimate(startPos);

        Control.auto.initTF("FreightFrenzy_DM.tflite", new String[]{
                "Duck",
                "Marker"
        }, 1.0, hardwareMap);
//        Control.sensor.initGyro();

        telemetry.addData(">", "Press Play to start op mode");
        telemetry.update();

        waitForStart();

        if (isStopRequested()) return;

        Recognition duck = Control.auto.getDuck(telemetry);

        double duckPositionAngle = duck.estimateAngleToObject(AngleUnit.DEGREES);
        int duckPositionIndex = Control.auto.getDuckPositionIndexThree(duckPositionAngle);

        telemetry.addData("duck position index: ", duckPositionIndex);
        telemetry.update();

        Trajectory traj1 = drive.trajectoryBuilder(startPos)
                .splineTo(new Vector2d(coeff * -11, coeff * -47), Math.toRadians(90))
                .build();
        drive.followTrajectory(traj1);

        // move to carousel
//        Control.auto.strafeToPosition(20, 1.0);
//
//        // get duck off of carousel
//        Control.auto.spinCarousel(Devices.carouselMotor);
//
//        // move to align with shipping hub
//        Control.auto.strafeToPosition(40, -1.0);

        // move towards shipping hub
//        Control.auto.moveWithEncoder(20, 1.0);

        // move cargo to level indicated by duck
        int targetPosition = duckPositionIndex == 1 || duckPositionIndex == 2 ? duckPositionIndex == 1 ? 200 : 1000 : 0;
        Control.motor.linearSlideSetPosition(Devices.armLiftMotor, targetPosition);
        // Control.motor.dumpCargo();

        Trajectory traj2 = drive.trajectoryBuilder(traj1.end())
                .strafeLeft(coeff * 24)
                .build();
        drive.followTrajectory(traj2);

        // pick up duck

        Trajectory traj3 = drive.trajectoryBuilder(traj2.end())
                .strafeRight(coeff * 24)
                .build();
        drive.followTrajectory(traj3);

        // Control.motor.dumpCargo();

        Trajectory traj4 = drive.trajectoryBuilder(traj3.end())
                .lineTo(new Vector2d(-60 * coeff, -60))
                .build();
        drive.followTrajectory(traj4);

        // get duck off of carousel
//        Control.auto.spinCarousel(Devices.carouselMotor);

        Trajectory traj5 = drive.trajectoryBuilder(traj4.end())
                .forward(25)
                .build();
        drive.followTrajectory(traj5);

        return;
    }
}
