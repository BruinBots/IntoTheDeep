package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.Servo;

public class Intake {
    /*
    INTAKE_SPEED
    Range: 0-1
    Controls the speed the intake moves during intaking and outtaking
     */
    public static double INTAKE_SPEED = 0.5;

    private boolean currentlyChecking = false;
    private long outTakeTime;

    private long timeToOuttake = 1000;

    private Servo leftServo;
    private Servo rightServo;

    public Intake(Servo left, Servo right) {
        this.leftServo = left;
        this.rightServo = right;
    }

    public void stop() {
        // Stops the intake mechanism
        setSpeed(0);
    }

    public void intake() {
        // Intakes the intake mechanism at INTAKE_SPEED
        setSpeed(INTAKE_SPEED);
    }

    public void outtake() {
        // Outtakes the intake mechanism (reverse of intake) at INTAKE_SPEED
        setSpeed(-INTAKE_SPEED);
    }

    public void setSpeed(double speed) {
        /*
        Move the intake servos at designated speed
        Range: 0            1
               Stopped      Fastest
         */
        double leftServoVal = 0.5 + (speed / 2);
        double rightServoVal = 0.5 - (speed / 2);

        leftServo.setPosition(leftServoVal);
        rightServo.setPosition(rightServoVal);
    }

    //outake for certain time
    public void outtakeForTime() {
        outtake();
        outTakeTime = getCurrentTime();
        currentlyChecking = true;
    }

    public void checkTime() {
        if (currentlyChecking) {
            if (getCurrentTime() >= outTakeTime + timeToOuttake) {
                currentlyChecking = false;
                stop();

            }
        }
    }

    private long getCurrentTime() {
        return System.currentTimeMillis();
    }
}
