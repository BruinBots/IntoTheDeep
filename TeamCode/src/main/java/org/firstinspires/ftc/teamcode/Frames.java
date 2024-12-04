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

            boolean cont = System.currentTimeMillis() - startTime >= waitTime;

            if (cont) {
                startTime = -1;
            }

            return cont;
        }
    }

    private class ViperFrame extends Frame {
        private int viper;
        private int delay;

        public ViperFrame(int viper, int delay) {
            this.viper = viper;
            this.delay = delay;
        }

        public ViperFrame(int viper) {
            this.viper = viper;
            this.delay = 500;
        }

        public boolean run() {
            bot.viper.move(viper);

            MainTeleop.viperPos = viper;

            return waitUntilTime(delay) && (Math.abs(bot.viperMotorL.getTargetPosition() - bot.viperMotorL.getCurrentPosition()) <= 15) && (Math.abs(bot.viperMotorR.getTargetPosition() - bot.viperMotorR.getCurrentPosition()) <= 15);
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
      new ArmWristSanityFrame(6709, 0.22, 5500, 0.3),
      new ArmWristFrame(6709, 0.22),
    };

    public Frame[] peckFrames = new Frame[] {
      new ArmWristSanityFrame(6709, 0.22, 1000, 0.2),
      new ArmWristFrame(7392, 0.22, 500),
      new ClawEngageFrame(),
      new ArmWristFrame(6709, 0.22),
    };

    public Frame[] clawToBasketFrames = new Frame[] {
      new ArmWristSanityFrame(3562, 0.26, 5500, 0.3),
        new ArmWristFrame(3362, 0.22),
        new ArmWristFrame(3362, 0.81),
        new BasketMidFrame(),
        new ArmWristFrame(2341, 0.81),
        new ClawStandbyFrame(),
        new ArmWristFrame(2341, 0.9)
    };

    public Frame[] afterGrabFrames = new Frame[] {
      new ArmWristSanityFrame(3362, 0.22, 5500, 0.3),
      new ArmWristFrame(3362, 0.22),
    };

    public Frame[] zeroBasketFrames = new Frame[] {
        new ViperFrame(0),
    };

    public Frame[] bottomBasketFrames = new Frame[] {
        new ViperFrame(1600),
    };

    public Frame[] topBasketFrames = new Frame[] {
        new ViperFrame(3200),
    };

    public Frame[] uncurlFrames = new Frame[] {
        new ArmWristSanityFrame(0, 1, 1000, 0.2),
        new ArmWristFrame(2025, 1),
        new ArmWristFrame(2025, 0.87),
        new ArmWristFrame(3576, 0.87),
        new ArmWristFrame(3576, 0.22),
    };

    public Frames(Hardware bot) {
        this.bot = bot;
    }

    public void runFrames(Frame[] frames) {
        if (curFrames == frames) {
            bot.arm.moveArm(bot.armMotor.getCurrentPosition()); // Stop moving arm by commanding it to move to its current position
            MainTeleop.armPos = bot.armMotor.getCurrentPosition();
            bot.viper.move(bot.viperMotorL.getCurrentPosition()); // Stop moving vipers
            MainTeleop.viperPos = bot.viperMotorL.getCurrentPosition();

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
    public void zeroBasket() { runFrames(zeroBasketFrames); }
    public void bottomBasket() { runFrames(bottomBasketFrames); }
    public void topBasket() { runFrames(topBasketFrames); }
    public void uncurlArm() { runFrames(uncurlFrames); }
}
