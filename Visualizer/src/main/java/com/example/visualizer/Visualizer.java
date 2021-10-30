package com.example.visualizer;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.noahbres.meepmeep.MeepMeep;
import com.noahbres.meepmeep.core.colorscheme.scheme.ColorSchemeRedDark;

public class Visualizer {
    public static void main(String[] args) {
        // TODO: If you experience poor performance, enable this flag
//         System.setProperty("sun.java2d.opengl", "true");

        // Declare a MeepMeep instance
        // With a field size of 800 pixels
        MeepMeep mm = new MeepMeep(800)
                // Set field image
                .setBackground(MeepMeep.Background.FIELD_FREIGHT_FRENZY)
                // Set theme
                .setTheme(new ColorSchemeRedDark())
                // Background opacity from 0-1
                .setBackgroundAlpha(1f)
                // Set constraints: maxVel, maxAccel, maxAngVel, maxAngAccel, track width
                .setConstraints(68.88, 68.88, Math.toRadians(180), Math.toRadians(180), 12.5)
                .followTrajectorySequence(drive ->
                        drive.trajectorySequenceBuilder(new Pose2d(-35  , -61, Math.toRadians(90)))
//                                .splineTo(new Vector2d(-35, -40), Math.toRadians(90))
//                                .splineTo(new Vector2d(-20, -40), Math.toRadians(90))
                                .splineTo(new Vector2d(-11, -47), Math.toRadians(90))
//                                .turn(Math.toRadians(90))
//                                .splineTo(new Vector2d(-60, -50), Math.toRadians(90))
//                                .splineTo(new Vector2d(-60, -60), Math.toRadians(90))
//                                .splineTo(new Vector2d(-35, -46), Math.toRadians(90))
                                .strafeLeft(24)
                                .strafeRight(24)
                                .strafeLeft(49)
                                .forward(-13)
                                .forward(25)
//                                .back(10)
//                                .forward(30)
//                                .turn(Math.toRadians(90))
//                                .forward(30)
//                                .turn(Math.toRadians(90))
//                                .splineTo(new Vector2d(-30, -55), Math.toRadians(90))
//                                .splineTo(new Vector2d(-60, -60), Math.toRadians(90))
                                .build()
                )
                .start();
    }
}
