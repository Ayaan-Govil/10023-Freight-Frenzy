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
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;

import org.firstinspires.ftc.robotcore.external.tfod.Recognition;
import org.firstinspires.ftc.teamcode.BaseRobot;
import org.firstinspires.ftc.teamcode.hardware.*;

@Autonomous

// basic autonomous that moves forward 10 inches, waits 5 seconds, turns around, then drives back

public class FreightFrenzyAuto extends LinearOpMode {

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
        Control.sensor.initTF("FreightFrenzy_DM.tflite", new String[]{
                "Duck",
                "Marker"
        }, 1.0, hardwareMap);
        Control.sensor.initGyro();


        waitForStart();

        Recognition duck = Control.sensor.getDuck(telemetry);
        double duckPositionAngle = duck.estimateAngleToObject(AngleUnit.DEGREES);
        int duckPositionIndex = Control.sensor.getDuckPositionIndex(duckPositionAngle);

        telemetry.addData("position: ", duckPositionIndex);
        telemetry.update();
//        Control.auto.moveWithEncoder(18, 1.0);

//        Control.auto.turnWithGyro(90, 0.5);
    }
}
