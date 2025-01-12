package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.Servo;

public class CarJack {

    Servo carJackServo;
    DigitalChannel limitSwitch;
    double goingUpPos = 1;
    double goingDownPos = 0;
    double stillPos = 0.5;

    public CarJack(Servo servo, DigitalChannel limitSwitch) {
        this.carJackServo = servo;
        this.limitSwitch = limitSwitch;
    }

    public void moveUp() {
        if (limitSwitch.getState()) { // if limit switch is pressed
            stop();
        } else {
            carJackServo.setPosition(goingUpPos);
        }
    }

    public void moveDown() {
        carJackServo.setPosition(goingDownPos);
    }

    public void stop() {
        carJackServo.setPosition(stillPos);
    }

}