package org.firstinspires.ftc.teamcode.autos;

import org.firstinspires.ftc.teamcode.BaseRobot;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;

import org.firstinspires.ftc.teamcode.hardware.*;

@Autonomous

// basic autonomous that moves forward 10 inches, waits 5 seconds, turns around, then drives back

public class FreightFrenzyAuto extends BaseRobot {

    @Override
    public void init() {
        super.init();
    }

    @Override
    public void start() {
        super.start();

        Control.auto.moveToPosition(10, 0.5);
    }

    @Override
    public void loop() {
        super.loop();
    }
}