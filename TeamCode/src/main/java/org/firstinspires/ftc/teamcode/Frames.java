package org.firstinspires.ftc.teamcode;

import static java.lang.Thread.sleep;

public class Frames {
    private abstract class Frame {
        public double startTime = -1;

        public Frame() { }

        public boolean sanity() { return true; }
        abstract public boolean run();
        public boolean waitUntilTime(int waitTime) {
            if (startTime < 0) {
                startTime = System.currentTimeMillis();
            }

            return System.currentTimeMillis() - startTime >= waitTime;
        }
    }

    private class ArmWristFrame extends Frame {
        private int arm;
        private double wrist;
        private int delay;

        public ArmWristFrame(int arm, double wrist) {
            this.arm = arm;
            this.wrist = wrist;
            this.delay = 500;
        }

        public ArmWristFrame(int arm, double wrist, int delay) {
            this.arm = arm;
            this.wrist = wrist;
            this.delay = delay;
        }

        public boolean run() {
            bot.arm.moveArm(arm);
            bot.arm.moveWrist(wrist);

            MainTeleop.armPos = arm;
            MainTeleop.wristPos = wrist;

            return waitUntilTime(delay) && (Math.abs(bot.armMotor.getTargetPosition() - bot.armMotor.getCurrentPosition()) <= 5);
        }
    }

    private class BasketMidFrame extends Frame {

        public boolean run() {
            bot.basket.setMiddle();

            return waitUntilTime(500);
        }
    }

    private class ClawStandbyFrame extends Frame {

        public boolean run() {
            bot.intake.standby();

            return waitUntilTime(500);
        }
    }

    private class ClawEngageFrame extends Frame {

        public boolean run() {
            bot.intake.engage();

            return waitUntilTime(500);
        }
    }

    private class ArmWristSanityFrame extends Frame {
        private int arm;
        private double wrist;
        private int armTolerance;
        private double wristTolerance;

        public ArmWristSanityFrame(int arm, double wrist, int armTolerance, double wristTolerance) {
            this.arm = arm;
            this.wrist = wrist;
            this.armTolerance = armTolerance;
            this.wristTolerance = wristTolerance;
        }

        public boolean run() { return true; }

        public boolean sanity() {
            int curArm = bot.armMotor.getCurrentPosition();
            double curWrist = bot.wristServo.getPosition();

            return Math.abs(arm - curArm) <= armTolerance && Math.abs(wrist - curWrist) <= wristTolerance;
        }
    }

    /*
    Arm -> Basket handoff positions:
    Arm     Wrist
    7297    0.279 <== BEFORE PICK UP
    ----------------------------------
    3362    0.264 <== AFTER PICK UP
    3362    0.832
    2359    0.832
    BASKET TO MID
    CLAW STANDBY (drop)
    2359    0.936 (final release)
    ---------------------------------
    3362    0.264 <== AFTER PICK UP
     */

    private Hardware bot;
    private Frame[] curFrames = new Frame[] {};
    private int curIdx = 0;

    public Frame[] beforeGrabFrames = new Frame[] {
      new ArmWristSanityFrame(6850, 0.279, 5500, 0.3),
      new ArmWristFrame(6850, 0.247),
    };

    public Frame[] peckFrames = new Frame[] {
      new ArmWristSanityFrame(6850, 0.247, 1000, 0.2),
      new ArmWristFrame(7343, 0.247, 1000),
      new ClawEngageFrame(),
      new ArmWristFrame(6850, 0.247),
    };

    public Frame[] clawToBasketFrames = new Frame[] {
      new ArmWristSanityFrame(3562, 0.264, 5500, 0.3),
      new ArmWristFrame(3562, 0.264),
      new ArmWristFrame(4304, 0.816, 3500),
      new ArmWristFrame(2862, 0.816),
      new ArmWristFrame(2359, 0.816),
      new BasketMidFrame(),
      new ClawStandbyFrame(),
      new ArmWristFrame(2459, 0.936),
    };

    public Frame[] afterGrabFrames = new Frame[] {
      new ArmWristSanityFrame(3362, 0.264, 5500, 0.3),
      new ArmWristFrame(3362, 0.264),
    };

    public Frames(Hardware bot) {
        this.bot = bot;
    }

    public void runFrames(Frame[] frames) {
        if (curFrames == frames) {
            bot.arm.moveArm(bot.armMotor.getCurrentPosition()); // Stop moving arm by commanding it to move to its current position
            curFrames = new Frame[] {};
            curIdx = 0;
            return;
        }
        curFrames = frames;
        curIdx = 0;
    }

    public boolean isBusy() {
        return curFrames.length > 0;
    }

    public void loop() {
        if (curFrames.length == 0) { return; }

        Frame curFrame = curFrames[curIdx];

        // Verify sanity checks
        if (!curFrame.sanity()) {
            // If sanity check fails, abort frames and reset
            curIdx = 0;
            curFrames = new Frame[] {};
            return;
        }

        // Run frame
        if (curFrame.run()) {
            // If run completed, progress to next frame, or complete frames and reset
            if (curIdx < curFrames.length - 1) {
                curIdx ++;
            }
            else {
                curIdx = 0;
                curFrames = new Frame[] {};
            }
        }
    }

    public void beforeGrab() { runFrames(beforeGrabFrames); }
    public void peck() { runFrames(peckFrames); }
    public void clawToBasket() { runFrames(clawToBasketFrames); }
    public void afterGrab() { runFrames(afterGrabFrames); }
}
