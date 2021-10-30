package org.firstinspires.ftc.teamcode.autos;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

@Autonomous

// basic autonomous that moves forward 10 inches, waits 5 seconds, turns around, then drives back

public class ThreeMarkerDuckBlue extends ThreeMarkerDuck {

    @Override
    public void runOpMode() {
        super.coeff = -1;
        super.runOpMode();
        return;
    }
}
