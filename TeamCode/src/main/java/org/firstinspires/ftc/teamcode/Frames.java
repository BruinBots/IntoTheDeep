package org.firstinspires.ftc.teamcode;

import org.apache.commons.math3.analysis.function.Constant;
import org.firstinspires.ftc.robotcore.external.Const;

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

            return waitUntilTime(0);
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

    private class ViperSanityFrame extends Frame {
        private int viperMin;
        private int viperMax;

        public ViperSanityFrame(int min, int max) {
            this.viperMin = min;
            this.viperMax = max;
        }

        public boolean run() {
            return true;
        }

        public boolean sanity() {
            int curViper = bot.viperMotorL.getCurrentPosition();

            return curViper >= viperMin && curViper <= viperMax;
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
            new ArmWristSanityFrame(6200, 0.22, 5500, 0.3),
            new ArmWristFrame(6200, 0.22),
    };

    public Frame[] peckFrames = new Frame[]{
            new ArmWristSanityFrame(6200, 0.22, 1000, 0.2),
            new ArmWristFrame(7500, 0.22, 500),
            new ClawEngageFrame(),
            new ArmWristFrame(6200, 0.22),
    };

    public Frame[] standbyFrames = new Frame[]{
            new ArmMinMaxFrame(3476, 99999),
            new ArmWristFrame(3362, 0.22),
    };

    public Frame[] zeroBasketFrames = new Frame[]{
            new ViperFrame(0, 1000),
    };

    public Frame[] topPoleFrames = new Frame[]{
            new ViperFrame(ChamberPlacer.startViper) // top pole pos
    };

    public Frame[] topSpecimenFrames = new Frame[] {
            new ViperSanityFrame(2200, 4000),
            new ViperFrame(ChamberPlacer.downViper, 100),
            new BasketOpenFrame(),
    };

    public Frame[] beforeChamberFrames = new Frame[] {
            new ViperFrame(ChamberPlacer.startViper, 50),
    };

    public Frame[] afterWallFrames = new Frame[] {
            new WaitFrame(500),
            new ViperFrame(WallPicker.afterWallPickerPos, 99999),
    };

    public Frame[] pickupFromWallFrames = new Frame[] {
            new ViperFrame(WallPicker.wallPickerPos, 20),
            new BasketCloseFrame(),
            new WaitFrame(500),
            new ViperFrame(WallPicker.afterWallPickerPos, 99999),
    };

    public Frame[] beforeWallFrames = new Frame[] {
            new BasketOpenFrame(),
            new ViperFrame(WallPicker.wallPickerPos, 50),
    };

    public Frame[] uncurlFrames = new Frame[]{
            new ArmWristSanityFrame(0, 1, 1000, 0.2),
            new ArmWristFrame(1320, 1, 0, 500),
            new ArmWristFrame(1320, 0.97, 100, 500),
            new ArmWristFrame(1970, 0.97, 0, 500),
            new ArmWristFrame(1970, 0.92, 100, 500),
            new ArmWristFrame(2624, 0.92, 0, 500),
            new ArmWristFrame(2624, 0.75, 100, 500),
            new ArmWristFrame(2624, 0.22, 0, 500),
            new ArmWristFrame(3374, 0.22, 100, 500),
    };

    public Frame[] curlFrames = new Frame[]{
            new ArmWristSanityFrame(3374, 0.22, 1000, 0.2),
            new ArmWristFrame(3374, 0.22, 100, 500),
            new ArmWristFrame(3374, 0.75, 800, 500),
            new ArmWristFrame(2624, 0.75, 0, 500),
            new ArmWristFrame(2624, 0.92, 100, 500),
            new ArmWristFrame(1970, 0.92, 0, 500),
            new ArmWristFrame(1970, 0.97, 100, 500),
            new ArmWristFrame(1320, 0.97, 0, 500),
            new ArmWristFrame(0, 1, 0, 500),
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

    public void stop() {
        this.curFrames = new Frame[]{};
        this.curIdx = 0;
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

    public void afterGrab() {
        runFrames(standbyFrames);
    }

    public void zeroBasket() {
        runFrames(zeroBasketFrames);
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


    public void afterWall() { runFrames(afterWallFrames); }

    public void pickupFromWall() { runFrames(pickupFromWallFrames); }

    public void beforeChamber() { runFrames(beforeChamberFrames); }

    public void beforeWall() { runFrames(beforeWallFrames); }
}