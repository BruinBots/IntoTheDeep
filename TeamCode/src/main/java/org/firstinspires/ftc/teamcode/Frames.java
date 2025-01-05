package org.firstinspires.ftc.teamcode;

import static java.lang.Thread.sleep;

public class Frames {
    private abstract class Frame {
        public double startTime = -1;

        public Frame() {
        }

        public boolean sanity() {
            return true;
        }

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
        private int tolerance;

        public ViperFrame(int viper, int tolerance) {
            this.viper = viper;
            this.tolerance = tolerance;
        }

        public ViperFrame(int viper) {
            this.viper = viper;
            this.tolerance = 15;
        }

        public boolean run() {
            bot.viper.move(viper, Viper.Sides.LEFT);

//            MainTeleop.viperPos = viper;

            return waitUntilTime(500) && (Math.abs(bot.viperMotorL.getTargetPosition() - bot.viperMotorL.getCurrentPosition()) <= tolerance);
        }
    }

    private class ArmWristFrame extends Frame {
        private int arm;
        private double wrist;
        private int delay = 500;
        private int tolerance = 5;

        public ArmWristFrame(int arm, double wrist) {
            this.arm = arm;
            this.wrist = wrist;
        }

        public ArmWristFrame(int arm, double wrist, int delay) {
            this.arm = arm;
            this.wrist = wrist;
            this.delay = delay;
        }

        public ArmWristFrame(int arm, double wrist, int delay, int tolerance) {
            this.arm = arm;
            this.wrist = wrist;
            this.delay = delay;
            this.tolerance = tolerance;
        }

        public boolean run() {
            bot.arm.moveArm(arm);
            bot.arm.moveWrist(wrist);

            MainTeleop.armPos = arm;
            MainTeleop.wristPos = wrist;

            return waitUntilTime(delay) && (Math.abs(bot.armMotor.getTargetPosition() - bot.armMotor.getCurrentPosition()) <= tolerance);
        }
    }

    private class BasketMidFrame extends Frame {

        public boolean run() {
//            bot.basket.setMiddle();

            return waitUntilTime(250);
        }
    }

    private class BasketOpenFrame extends Frame {

        public boolean run() {
            bot.basket.setOpen();

            return waitUntilTime(250);
        }

    }

    private class BasketCloseFrame extends Frame {

        public boolean run() {
            bot.basket.setClosed();

            return waitUntilTime(250);
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

        public boolean run() {
            return true;
        }

        public boolean sanity() {
            int curArm = bot.armMotor.getCurrentPosition();
            double curWrist = bot.wristServo.getPosition();

            return Math.abs(arm - curArm) <= armTolerance && Math.abs(wrist - curWrist) <= wristTolerance;
        }
    }

    public class ArmMinMaxFrame extends Frame {
        private int max;
        private int min;

        public ArmMinMaxFrame(int min, int max) {
            this.min = min;
            this.max = max;
        }

        public boolean run() {
            return true;
        }

        public boolean sanity() {
            int curArm = bot.armMotor.getCurrentPosition();

            return curArm >= min && curArm <= max;
        }
    }

    public class WaitFrame extends Frame {
        private int ms;

        public WaitFrame(int ms) {
            this.ms = ms;
        }

        public boolean run() {
            return waitUntilTime(ms);
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
    private Frame[] curFrames = new Frame[]{};
    private int curIdx = 0;
    private double lastTime = -1;

    public Frame[] beforeGrabFrames = new Frame[]{
            new ArmWristSanityFrame(5930, 0.22, 5500, 0.3),
            new ArmWristFrame(5930, 0.22),
    };

    public Frame[] peckFrames = new Frame[]{
            new ArmWristSanityFrame(5930, 0.22, 1000, 0.2),
            new ArmWristFrame(7239, 0.22, 500),
            new ClawEngageFrame(),
            new ArmWristFrame(5930, 0.22),
    };

    public Frame[] specimenFrames = new Frame[]{
            new ArmWristSanityFrame(5930, 0.22, 1000, 0.2),
            new ArmWristFrame(5930, 0.11),
            new ClawEngageFrame(),
            new ArmWristFrame(5000, 0.11),
    };

    public Frame[] clawToBasketFrames = new Frame[]{
            new ArmWristSanityFrame(4220, 0.22, 5500, 0.3),
            new ArmWristFrame(4220, 0.22, 250),
            new ArmWristFrame(4220, 0.79, 250),
            new ArmWristFrame(3640, 0.79, 100, 100),
            new BasketMidFrame(),
            new ArmWristFrame(2870, 0.79, 100, 100),
            new ArmWristFrame(2650, 0.84, 100, 100),
            new ClawStandbyFrame(),
            new ArmWristFrame(2650, 0.8, 250),
            new ArmWristFrame(2650, 0.86, 250),
            new ArmWristFrame(2650, 0.8, 250),
            new ArmWristFrame(2650, 0.86, 250),
//        new BasketCloseFrame(),
    };

    public Frame[] standbyFrames = new Frame[]{
            new ArmMinMaxFrame(3476, 99999),
            new ArmWristFrame(3362, 0.22),
    };

    public Frame[] zeroBasketFrames = new Frame[]{
            new ViperFrame(0, 1000),
    };

    public Frame[] bottomBasketFrames = new Frame[]{
            new ViperFrame(3630),
//        new BasketOpenFrame(),
    };

    public Frame[] topBasketFrames = new Frame[]{
            new ViperFrame(6100),
            new BasketOpenFrame(),
    };

    public Frame[] topPoleFrames = new Frame[]{
            new ViperFrame(3050) // top pole pos
    };

    public Frame[] topSpecimenFrames = new Frame[] {
            new ViperFrame(1950, 5),
            new BasketOpenFrame(),
    };

    public Frame[] afterWallFrames = new Frame[] {
            new WaitFrame(500),
            new ViperFrame(1500, 5),
    };

    public Frame[] uncurlFrames = new Frame[]{
            new ArmWristSanityFrame(0, 1, 1000, 0.2),
            new ArmWristFrame(1478, 1, 0, 500),
            new ArmWristFrame(1478, 0.96, 100, 500),
            new ArmWristFrame(2177, 0.96, 0, 500),
            new ArmWristFrame(2177, 0.92, 100, 500),
            new ArmWristFrame(2748, 0.92, 0, 500),
            new ArmWristFrame(2748, 0.86, 100, 500),
            new ArmWristFrame(3506, 0.86, 0, 500),
            new ArmWristFrame(3506, 0.2, 100, 500),
    };

    public Frame[] curlFrames = new Frame[]{
            new ArmWristSanityFrame(3506, 0.2, 1000, 0.2),
            new ArmWristFrame(4000, 0.2, 250),
            new ArmWristFrame(4000, 0.85, 1000),
            new ArmWristFrame(2920, 0.85, 0, 500),
            new ArmWristFrame(2920, 0.91, 100, 500),
            new ArmWristFrame(2040, 0.91, 0, 500),
            new ArmWristFrame(2040, 1, 100, 500),
            new ArmWristFrame(400, 1, 0, 500),
    };

    public Frame[] liftFrames = new Frame[]{
            new ArmWristFrame(400, 1)
    };

    public Frame[] zeroArmFrames = new Frame[]{
            new ArmWristFrame(0, 1, 250, 5)
    };

    public Frames(Hardware bot) {
        this.bot = bot;
    }

    public void runFrames(Frame[] frames) {
        if (System.currentTimeMillis() - lastTime <= 500) {
            return;
        }

        lastTime = System.currentTimeMillis();

        if (curFrames == frames) {
            bot.arm.moveArm(bot.armMotor.getCurrentPosition()); // Stop moving arm by commanding it to move to its current position
            MainTeleop.armPos = bot.armMotor.getCurrentPosition();
            bot.viper.move(bot.viperMotorL.getCurrentPosition()); // Stop moving vipers
//            MainTeleop.viperPos = bot.viperMotorL.getCurrentPosition();

            curFrames = new Frame[]{};
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
        if (curFrames.length == 0) {
            return;
        }

        Frame curFrame = curFrames[curIdx];

        // Verify sanity checks
        if (!curFrame.sanity()) {
            // If sanity check fails, abort frames and reset
            curIdx = 0;
            curFrames = new Frame[]{};
            return;
        }

        // Run frame
        if (curFrame.run()) {
            // If run completed, progress to next frame, or complete frames and reset
            if (curIdx < curFrames.length - 1) {
                curIdx++;
            } else {
                curIdx = 0;
                curFrames = new Frame[]{};
            }
        }
    }

    public void wait(int ms) {
        runFrames(new Frame[]{new WaitFrame(ms)});
    }

    public void beforeGrab() {
        runFrames(beforeGrabFrames);
    }

    public void peck() {
        runFrames(peckFrames);
    }

    public void clawToBasket() {
        runFrames(clawToBasketFrames);
    }

    public void afterGrab() {
        runFrames(standbyFrames);
    }

    public void zeroBasket() {
        runFrames(zeroBasketFrames);
    }

    public void bottomBasket() {
        runFrames(bottomBasketFrames);
    }

    public void topBasket() {
        runFrames(topBasketFrames);
    }

    public void topPole() {
        runFrames(topPoleFrames);
    }

    public void topSpecimen() { runFrames(topSpecimenFrames); }

    public void uncurlArm() {
        runFrames(uncurlFrames);
    }

    public void curlArm() {
        runFrames(curlFrames);
    }

    public void lift() {
        runFrames(liftFrames);
    }

    public void zeroArm() {
        runFrames(zeroArmFrames);
    }

    public void afterWall() { runFrames(afterWallFrames); }
}