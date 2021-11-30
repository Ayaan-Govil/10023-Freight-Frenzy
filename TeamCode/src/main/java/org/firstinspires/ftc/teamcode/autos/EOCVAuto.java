package org.firstinspires.ftc.teamcode.autos;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.hardware.Control;
import org.firstinspires.ftc.teamcode.hardware.Devices;
import org.firstinspires.ftc.teamcode.hardware.Encoders;
import org.firstinspires.ftc.teamcode.vision.AprilTagDetectionPipeline;
import org.openftc.apriltag.AprilTagDetection;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;

import java.util.ArrayList;

// TODO
// install .so file to control hub

// sim can be downloaded at https://github.com/deltacv/EOCV-Sim/releases

@Autonomous
public class EOCVAuto extends LinearOpMode {
    OpenCvCamera camera;
    AprilTagDetectionPipeline aprilTagDetectionPipeline;

    static final double FEET_PER_METER = 3.28084;

    // Lens intrinsics
    // UNITS ARE PIXELS
    // NOTE: this calibration is for the C920 webcam at 800x448.
    // You will need to do your own calibration for other configurations!
    double fx = 578.272;
    double fy = 578.272;
    double cx = 402.145;
    double cy = 221.506;

    // UNITS ARE METERS
    double tagsize = 0.173;

    int numFramesWithoutDetection = 0;

    final float DECIMATION_HIGH = 3;
    final float DECIMATION_LOW = 2;
    final float THRESHOLD_HIGH_DECIMATION_RANGE_METERS = 1.0f;
    final int THRESHOLD_NUM_FRAMES_NO_DETECTION_BEFORE_LOW_DECIMATION = 4;

    boolean foundTag = false;
    int pieceIndex = 0;

    SampleMecanumDrive drive;
    Pose2d startPos = new Pose2d(0, 0, Math.toRadians(90));

    Trajectory moveToCarousel;
    Trajectory park;

//    DcMotor lift;

    @Override
    public void runOpMode() {
        Devices.initDevices(hardwareMap);
//        lift = hardwareMap.get(DcMotor.class, "lift");
//        lift.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        camera = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, "Webcam 1"), cameraMonitorViewId);
        aprilTagDetectionPipeline = new AprilTagDetectionPipeline(tagsize, fx, fy, cx, cy);

        camera.setPipeline(aprilTagDetectionPipeline);
        camera.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener() {
            @Override
            public void onOpened() {
                camera.startStreaming(800, 448, OpenCvCameraRotation.UPRIGHT);
            }

            @Override
            public void onError(int errorCode) {

            }
        });

        initRR();

        telemetry.addLine("Ready to start!");
        telemetry.update();

        waitForStart();

        ElapsedTime timer = new ElapsedTime();

        telemetry.setMsTransmissionInterval(50);

        while (!foundTag) {

            if (timer.seconds() > 10) {
                foundTag = true;
                pieceIndex = 2;
                timer.reset();
            }
            // Calling getDetectionsUpdate() will only return an object if there was a new frame
            // processed since the last time we called it. Otherwise, it will return null. This
            // enables us to only run logic when there has been a new frame, as opposed to the
            // getLatestDetections() method which will always return an object.
            ArrayList<AprilTagDetection> detections = aprilTagDetectionPipeline.getDetectionsUpdate();

            // If there's been a new frame...
            if (detections != null) {
                telemetry.addData("FPS", camera.getFps());
                telemetry.addData("Overhead ms", camera.getOverheadTimeMs());
                telemetry.addData("Pipeline ms", camera.getPipelineTimeMs());

                // If we don't see any tags
                if (detections.size() == 0) {
                    numFramesWithoutDetection++;

                    // If we haven't seen a tag for a few frames, lower the decimation
                    // so we can hopefully pick one up if we're e.g. far back
                    if (numFramesWithoutDetection >= THRESHOLD_NUM_FRAMES_NO_DETECTION_BEFORE_LOW_DECIMATION) {
                        aprilTagDetectionPipeline.setDecimation(DECIMATION_LOW);
                    }
                }
                // We do see tags!
                else {
                    numFramesWithoutDetection = 0;

                    // If the target is within 1 meter, turn on high decimation to
                    // increase the frame rate
                    if (detections.get(0).pose.z < THRESHOLD_HIGH_DECIMATION_RANGE_METERS) {
                        aprilTagDetectionPipeline.setDecimation(DECIMATION_HIGH);
                    }

                    for (AprilTagDetection detection : detections) {
                        telemetry.addLine(String.format("\nDetected tag ID=%d", detection.id));
                        telemetry.addLine(String.format("Translation X: %.2f feet", detection.pose.x * FEET_PER_METER));
                        telemetry.addLine(String.format("Translation Y: %.2f feet", detection.pose.y * FEET_PER_METER));
                        telemetry.addLine(String.format("Translation Z: %.2f feet", detection.pose.z * FEET_PER_METER));
                        telemetry.addLine(String.format("Rotation Yaw: %.2f degrees", Math.toDegrees(detection.pose.yaw)));
                        telemetry.addLine(String.format("Rotation Pitch: %.2f degrees", Math.toDegrees(detection.pose.pitch)));
                        telemetry.addLine(String.format("Rotation Roll: %.2f degrees", Math.toDegrees(detection.pose.roll)));
                        telemetry.addData("center x: ", detection.center.x);
                        telemetry.addData("center y: ", detection.center.y);

                        double x = detection.center.x;

                        if (x < 250) pieceIndex = 0;
                        else if (x > 550) pieceIndex = 2;
                        else pieceIndex = 1;

                        foundTag = true;
                    }
                }

                telemetry.update();
            }

            sleep(20);
        }

        telemetry.addData("index: ", pieceIndex);
        telemetry.update();


//        int targetY = -47;
//
//        if (pieceIndex == 0) targetY = -50;
//        else if (pieceIndex == 1) targetY = -48;
//        else Control.motor.linearSlideSetPosition(Devices.armLiftMotor, 1000);

        Trajectory moveToHub = drive.trajectoryBuilder(startPos)
                .lineTo(new Vector2d(-15, 17))
//                .splineTo(new Vector2d(-57, -40), Math.toRadians(90))
//                .splineTo(new Vector2d(-23, -8), Math.toRadians(310))
                .build();
        drive.followTrajectory(moveToHub);

        Control.motor.linearSlideSetPosition(Devices.armLiftMotor, 800);

        telemetry.addLine("going up");
//            telemetry.update();


//        DcMotor motor = Devices.armLiftMotor;

//        telemetry.addData("going to position: ", 800);
//        double power = 1.0;
////        if (Encoders.getMotorEnc(motor) > 800) power = -1.0;
//
//        lift.setTargetPosition(800);
//        lift.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//        lift.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
//
//        lift.setPower(power);
//        while (lift.isBusy()) {
//
//        }
//        lift.setPower(0);
        // dump block

//        Trajectory moveToCarousel = drive.trajectoryBuilder(startPos)
//                // need to reverse
//                .splineTo(new Vector2d(-57, -40), Math.toRadians(90))
//                .splineTo(new Vector2d(-57, -60), Math.toRadians(135))
//                .build();
//        drive.followTrajectory(moveToCarousel);

//        Devices.dumpyServo.setPosition(0.6);
//
//        ElapsedTime timer = new ElapsedTime();
//
//        while (timer.seconds() > 2) {
//
//        }
//
////        drive.followTrajectory(moveToCarousel);
//
//        Devices.carouselServo.setPower(1.0);
//
//        timer.reset();
//
//        Devices.dumpyServo.setPosition(1.0);
//        Control.motor.linearSlideSetPosition(Devices.armLiftMotor, 0);
//
//        while (timer.seconds() > 5) {
//
//        }

//        drive.followTrajectory(park);
    }

    public void initRR() {
        drive = new SampleMecanumDrive(hardwareMap);
        drive.setPoseEstimate(startPos);

        moveToCarousel = drive.trajectoryBuilder(startPos)
                .splineTo(new Vector2d(-50, -60), Math.toRadians(110))
                .build();
        park = drive.trajectoryBuilder(startPos)
                .splineTo(new Vector2d(-60, -35), Math.toRadians(90))
                .build();
    }
}
