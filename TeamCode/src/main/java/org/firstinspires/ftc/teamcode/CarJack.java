package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DigitalChannel;

public class CarJack {

    CRServo carJackServo;
    DigitalChannel limitSwitch;
    double goingUpPower = 1;
    double goingDownPower = -1;
    double stillPower = 0;

    public CarJack(CRServo servo, DigitalChannel limitSwitch) {
        this.carJackServo = servo;
        this.limitSwitch = limitSwitch;
    }

    public void moveUp() {
        if (limitSwitch.getState()) { // if limit switch is pressed
            stop();
        } else {
            carJackServo.setPower(goingUpPower);
        }
    }

    public void moveDown() {
        carJackServo.setPower(goingDownPower);
    }

    public void stop() {
        carJackServo.setPower(stillPower);
    }

}