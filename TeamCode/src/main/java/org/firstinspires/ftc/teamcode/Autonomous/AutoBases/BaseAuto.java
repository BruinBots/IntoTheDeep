package org.firstinspires.ftc.teamcode.Autonomous.AutoBases;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.Hardware;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;

import java.util.Vector;

public class BaseAuto {
    private final HardwareMap hardwareMap;
    private final Telemetry telemetry;
    private final Hardware bot;
    public SampleMecanumDrive drive;
    public boolean blue;

    public BaseAuto(HardwareMap hardwareMap, Telemetry telemetry, Pose2d startingPosition, boolean blue) {
        this.hardwareMap = hardwareMap;
        this.telemetry = telemetry;
        this.blue = blue;

        bot = new Hardware(hardwareMap);

        drive = new SampleMecanumDrive(hardwareMap);
        drive.setPoseEstimate(startingPosition);
    }

    public static Vector2d pose2vector(Pose2d pose) {
        return new Vector2d(pose.getX(), pose.getY());
    }

    // ----- PARKING ----- //
    public Vector2d park_v() {
        return new Vector2d(0, 0);
    }

    public Trajectory park(Trajectory startTraj) {
        return park(startTraj.end());
    }

    public Trajectory park(Pose2d startPose) {
        Trajectory traj = drive.trajectoryBuilder(startPose)
                .lineToConstantHeading(park_v())
                .build();
        drive.followTrajectory(traj);
        return traj;
    }

    // ----- BASKET ----- //
    public Vector2d basket_v() { return new Vector2d(0, 0); }

    public Trajectory basket(Trajectory startTraj) {
        return basket(startTraj.end());
    }

    public Trajectory basket(Pose2d startPose) {
        Trajectory traj = drive.trajectoryBuilder(startPose)
                .lineToConstantHeading(basket_v())
                .build();
        drive.followTrajectory(traj);

        // TODO: Place sample in basket

        return traj;
    }
}
