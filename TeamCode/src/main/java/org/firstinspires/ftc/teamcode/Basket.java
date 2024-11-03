package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.Servo;

public class Basket {

    private Servo basketServo;

    private double CLOSED_POS = 0.45;
    private double OPEN_POS = 0.7;
    private double MID_POS = 0.6;

    public Basket (Servo servo) {
        basketServo = servo;
    }

    private void setPosition (double position) {
        basketServo.setPosition(position);
    }

    public void getPosition () {
        basketServo.getPosition();
    }

    public void setClosed () {
        setPosition(CLOSED_POS);
    }

    public void setOpen() {
        setPosition(OPEN_POS);
    }

    public void setMiddle() {
        setPosition(MID_POS);
    }

}
