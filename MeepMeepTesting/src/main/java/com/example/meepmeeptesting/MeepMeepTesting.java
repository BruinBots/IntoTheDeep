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
                .setConstraints(45, 45, Math.toRadians(360), Math.toRadians(360), 14.35)
                .followTrajectorySequence(drive -> drive.trajectorySequenceBuilder(new Pose2d(-36, -60, Math.toRadians(90)))
                        .lineToLinearHeading(new Pose2d(-52, -52, Math.toRadians(225)))
                        .back(2)
                        .lineToConstantHeading(new Vector2d(-48, -10))
                        .lineToLinearHeading(new Pose2d(-30, -6, Math.toRadians(0)))
                        .build());


        meepMeep.setBackground(MeepMeep.Background.FIELD_INTOTHEDEEP_JUICE_DARK)
                .setDarkMode(true)
                .setBackgroundAlpha(0.95f)
                .addEntity(myBot)
                .start();
    }
}