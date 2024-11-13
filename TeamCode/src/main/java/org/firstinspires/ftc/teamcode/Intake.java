package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.Servo;

public class Intake {
    private Servo nearServo;
    private Servo farServo;
    public static double MAX_POS = 1;
    public static double MIN_POS = 0;
    public static final double NEAR_STANDBY = 0;
    public static final double FAR_STANDBY = 0.5;
    public static final double NEAR_ENGAGE = 0.15;
    public static final double FAR_ENGAGE = 0.4;

    public Intake(Servo near, Servo far) {
        this.nearServo = near;
        this.farServo = far;
    }

    public void engage(){
        nearServo.setPosition(NEAR_ENGAGE);
        farServo.setPosition(FAR_ENGAGE);
    }

    public void standby(){
        nearServo.setPosition(NEAR_STANDBY);
        farServo.setPosition(FAR_STANDBY);
    }


    public double getNearPos() { return (double) nearServo.getPosition(); }

    public double getFarPos(){
        return (double) farServo.getPosition();
    }

}
