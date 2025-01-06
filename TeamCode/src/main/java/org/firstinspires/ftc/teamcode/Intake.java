package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.Servo;

@Config
public class Intake {
    private Servo servo;
    private static double STANDBY = 0;
    private static double ENGAGE = 0.18;

    public Intake(Servo servo) {
        this.servo = servo;
    }

    public void engage() {
        servo.setPosition(ENGAGE);
    }

    public void standby() {
        servo.setPosition(STANDBY);
    }


    public double getPosition() {
        return servo.getPosition();
    }

}
