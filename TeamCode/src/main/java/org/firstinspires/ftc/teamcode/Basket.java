package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.Servo;

@Config
public class Basket {

    private Servo basketServo;

    public static double CLOSED_POS = 0.3;
    public static double OPEN_POS = 0;

    public Basket(Servo servo) {
        basketServo = servo;
    }

    private void setPosition(double position) {
        basketServo.setPosition(position);
    }

    public double getPosition() {
        return basketServo.getPosition();
    }

    public void setClosed() {
        setPosition(CLOSED_POS);
    }

    public void setOpen() {
        setPosition(OPEN_POS);
    }
}
