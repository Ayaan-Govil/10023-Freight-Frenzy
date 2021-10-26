package org.firstinspires.ftc.teamcode.teleops;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import java.util.List;

import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.tfod.TFObjectDetector;
import org.firstinspires.ftc.robotcore.external.tfod.Recognition;
import org.firstinspires.ftc.teamcode.hardware.ConstantVariables;
import org.firstinspires.ftc.teamcode.hardware.Control;
import org.firstinspires.ftc.teamcode.hardware.Devices;

/**
 * This 2020-2021 OpMode illustrates the basics of using the TensorFlow Object Detection API to
 * determine the position of the Freight Frenzy game elements.
 * <p>
 * Use Android Studio to Copy this Class, and Paste it into your team's code folder with a new name.
 * Remove or comment out the @Disabled line to add this opmode to the Driver Station OpMode list.
 * <p>
 * IMPORTANT: In order to use this OpMode, you need to obtain your own Vuforia license key as
 * is explained below.
 */
@TeleOp(name = "Concept: TensorFlow Object Detection Webcam", group = "Concept")
public class TensorflowSample extends LinearOpMode {

    @Override
    public void runOpMode() {
        Devices.initDevices(hardwareMap);
        Control.auto.initTF("FreightFrenzy_DM.tflite", new String[]{
                "Duck",
                "Marker"
        }, 1.0, hardwareMap);
        Control.sensor.initGyro();

        telemetry.addData(">", "Press Play to start op mode");
        telemetry.update();

        waitForStart();

        while (opModeIsActive()) {
//            if (tfod != null) {
//                // getUpdatedRecognitions() will return null if no new information is available since
//                // the last time that call was made.
//                List<Recognition> updatedRecognitions = tfod.getUpdatedRecognitions();
//                if (updatedRecognitions != null) {
//                    telemetry.addData("# Object Detected", updatedRecognitions.size());
//                    // step through the list of recognitions and display boundary info.
//                    int i = 0;
//                    for (Recognition recognition : updatedRecognitions) {
//                        telemetry.addData(String.format("label (%d)", i), recognition.getLabel());
//                        telemetry.addData(String.format("  left,top (%d)", i), "%.03f , %.03f",
//                                recognition.getLeft(), recognition.getTop());
//                        telemetry.addData(String.format("  right,bottom (%d)", i), "%.03f , %.03f",
//                                recognition.getRight(), recognition.getBottom());
//                        telemetry.addData("angle: ", recognition.estimateAngleToObject(AngleUnit.DEGREES));
//                        i++;
//                    }
//                    telemetry.update();
//                }
//            }
        }
    }
}
