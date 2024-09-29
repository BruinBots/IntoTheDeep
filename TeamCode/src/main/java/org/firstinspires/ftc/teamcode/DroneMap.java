package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

public class DroneMap {
    Servo droneReleaseServo;
    Servo droneRotateServo;
    Servo turretServo;
    public Drone drone;

    public DroneMap(HardwareMap hardwareMap) {
        droneReleaseServo = hardwareMap.get(Servo.class, "drone_release_servo");
        droneRotateServo = hardwareMap.get(Servo.class, "drone_rotate_servo");
        turretServo  =hardwareMap.get(Servo.class, "turret_servo");
        drone = new Drone(droneReleaseServo, droneRotateServo, turretServo);
    }
}
