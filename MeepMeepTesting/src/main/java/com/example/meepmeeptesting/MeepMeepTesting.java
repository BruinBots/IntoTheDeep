package com.example.meepmeeptesting;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;

import org.rowlandhall.meepmeep.MeepMeep;
import org.rowlandhall.meepmeep.core.colorscheme.ColorScheme;
import org.rowlandhall.meepmeep.core.colorscheme.scheme.ColorSchemeBlueDark;
import org.rowlandhall.meepmeep.core.colorscheme.scheme.ColorSchemeRedDark;
import org.rowlandhall.meepmeep.roadrunner.DefaultBotBuilder;
import org.rowlandhall.meepmeep.roadrunner.entity.RoadRunnerBotEntity;

public class MeepMeepTesting {
    public static double turnRelative(double curAngle, double targetAngle) {
        double deltaAngle = targetAngle - curAngle;

//        double plusAngle = targetAngle - (curAngle + 360);
//        double minusAngle = targetAngle - (curAngle - 360);

        double plusAngle = (targetAngle + Math.PI*2) - curAngle;
        double minusAngle = (targetAngle - Math.PI*2) - curAngle;

        return Math.min(Math.abs(deltaAngle), Math.min(Math.abs(plusAngle), Math.abs(minusAngle)));
    }

    public static double turnToCoords(double x1, double y1, double x2, double y2) {
        double dx = x2 - x1;
        double dy = y2 - y1;
        double tan = dy / dx;
        return Math.atan2(dy, dx);
    }

    public static void main(String[] args) {
        MeepMeep meepMeep = new MeepMeep(800);

        RoadRunnerBotEntity myBot = new DefaultBotBuilder(meepMeep)
                .setColorScheme(new ColorSchemeRedDark())
                // Set bot constraints: maxVel, maxAccel, maxAngVel, maxAngAccel, track width
                .setConstraints(40, 40, Math.toRadians(180), Math.toRadians(180), 14.35)
                .followTrajectorySequence(drive -> drive.trajectorySequenceBuilder(new Pose2d(10, -60, Math.toRadians(90)))
                        .forward(6)
                        .splineTo(new Vector2d(0, -42), Math.toRadians(90))
//                        .waitSeconds(6)
//                        .back(6)
//                        .turn(Math.toRadians(-90))
//                        .splineTo(new Vector2d(45, -54), Math.toRadians(270))
//                        .waitSeconds(3)
//                        .back(6)
//                        .turn(Math.toRadians(-90))
//                        .splineTo(new Vector2d(0, -42), Math.toRadians(90))
                        .turn(Math.toRadians(-90))
                        .forward(6)
                        .splineTo(new Vector2d(36, -12), Math.toRadians(90))
                        .splineTo(new Vector2d(46, -12), Math.toRadians(270))
                        .forward(40)
                        .build());


        meepMeep.setBackground(MeepMeep.Background.FIELD_INTOTHEDEEP_JUICE_DARK)
                .setDarkMode(true)
                .setBackgroundAlpha(0.95f)
                .addEntity(myBot)
                .start();
    }
}