package org.firstinspires.ftc.teamcode.NextYearTesting.SBAs;

import com.qualcomm.robotcore.hardware.Servo;

public class ServoSBA implements SBA {
    private Servo servo;
    private int targetPos;

    public ServoSBA(Servo servo, double power, int targetPos) {
        this.servo = servo;
        this.targetPos = targetPos;
    }

    public boolean run() {
        // Move servo to target position
        servo.setPosition(targetPos);
        return true;
    }

    public boolean sanity() { return true; }
}
