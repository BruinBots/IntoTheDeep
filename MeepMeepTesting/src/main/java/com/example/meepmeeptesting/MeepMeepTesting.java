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

        double plusAngle = (targetAngle + 360) - curAngle;
        double minusAngle = (targetAngle - 360) - curAngle;

        return Math.toRadians(Math.min(Math.abs(deltaAngle), Math.min(Math.abs(plusAngle), Math.abs(minusAngle))));
    }

    public static double turnToCoords(double x1, double y1, double x2, double y2) {
        double dx = x2 - x1;
        double dy = y2 - y1;
        double tan = dy / dx;
        return Math.toDegrees(Math.atan(tan));
    }

    public static void main(String[] args) {
        MeepMeep meepMeep = new MeepMeep(800);

        RoadRunnerBotEntity myBot = new DefaultBotBuilder(meepMeep)
                .setColorScheme(new ColorSchemeBlueDark())
                // Set bot constraints: maxVel, maxAccel, maxAngVel, maxAngAccel, track width
                .setConstraints(60, 60, Math.toRadians(180), Math.toRadians(180), 15)
                .followTrajectorySequence(drive -> drive.trajectorySequenceBuilder(new Pose2d(34, 60, Math.toRadians(270)))
                        .forward(3)
                        .turn(turnRelative(270, turnToCoords(34, 60, 48, 48)))
                        .lineToConstantHeading(new Vector2d(48, 48))
                        .turn(turnRelative(315, 45))
                        .build());


        meepMeep.setBackground(MeepMeep.Background.FIELD_INTOTHEDEEP_JUICE_DARK)
                .setDarkMode(true)
                .setBackgroundAlpha(0.95f)
                .addEntity(myBot)
                .start();
    }
}