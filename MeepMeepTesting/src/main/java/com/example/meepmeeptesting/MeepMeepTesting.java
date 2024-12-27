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

        double chamberTime = 0.5;
        double wallTime = 0.25;

        double firstTurn = turnRelative(Math.toRadians(90), turnToCoords(10, -54, 0, -32));
        double secondTurn = turnRelative(Math.toRadians(270), turnToCoords(0, -38, 48, -56));
        double thirdTurn = turnRelative(turnToCoords(0, -38, 48, -56), Math.toRadians(90));
        double fourthTurn = turnRelative(Math.toRadians(90), turnToCoords(48, -56, 0, -32))+Math.PI;

        RoadRunnerBotEntity myBot = new DefaultBotBuilder(meepMeep)
                .setColorScheme(new ColorSchemeRedDark())
                // Set bot constraints: maxVel, maxAccel, maxAngVel, maxAngAccel, track width
                .setConstraints(45, 45*4, Math.toRadians(270), Math.toRadians(270*4), 15)
                .followTrajectorySequence(drive -> drive.trajectorySequenceBuilder(new Pose2d(10, -60, Math.toRadians(270)))
                        .back(6)
                        .turn(firstTurn)
                        .lineToConstantHeading(new Vector2d(0, -32))
                        .turn(-firstTurn)
                        .waitSeconds(chamberTime)
                        // SPECIMEN -> HIGH CHAMBER
                        .forward(6)
                        .turn(secondTurn)
                        .lineToConstantHeading(new Vector2d(48, -56))
                        .turn(thirdTurn)
                        .back(6)
                        .waitSeconds(wallTime)
                        // SPECIMEN -> MAILBOX
                        // APRIL TAG
                        .forward(6)
                        .turn(fourthTurn)
                        .lineToConstantHeading(new Vector2d(0, -32))
                        .turn(-fourthTurn+Math.PI)
                        .waitSeconds(chamberTime)
                        // SPECIMEN -> HIGH CHAMBER
                        .forward(10)
                        .turn(Math.toRadians(90))
                        .lineToConstantHeading(new Vector2d(36, -42))
                        .turn(Math.toRadians(90))
                        .lineToConstantHeading(new Vector2d(36, 0))
                        .turn(Math.toRadians(-90))
                        .lineToConstantHeading(new Vector2d(42, 0))
                        .turn(Math.toRadians(-90))
                        .forward(48)
                        .back(48)
                        .turn(Math.toRadians(90))
                        .lineToConstantHeading(new Vector2d(52, 0))
                        .turn(Math.toRadians(-90))
                        .forward(48)
                        .back(48)
                        .turn(Math.toRadians(90))
                        .lineToConstantHeading(new Vector2d(62, 0))
                        .turn(Math.toRadians(-90))
                        .forward(48)
                        .build());


        meepMeep.setBackground(MeepMeep.Background.FIELD_INTOTHEDEEP_JUICE_DARK)
                .setDarkMode(true)
                .setBackgroundAlpha(0.95f)
                .addEntity(myBot)
                .start();
    }
}