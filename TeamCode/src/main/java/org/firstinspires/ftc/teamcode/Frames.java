package org.firstinspires.ftc.teamcode;

import static java.lang.Thread.sleep;

public class Frames {
    private abstract class Frame {
        public double startTime = -1;
        public double waitTime = 0;

        public Frame() { }

        public boolean sanity() { return true; }
        abstract public boolean run();
        public boolean waitUntilTime() {
            if (startTime < 0) {
                startTime = System.currentTimeMillis();
            }

            return System.currentTimeMillis() - startTime >= waitTime;
        }
    }

    private class ArmWristFrame extends Frame {
        private int arm;
        private double wrist;
        public double waitTime = 500;

        public ArmWristFrame(int arm, double wrist) {
            this.arm = arm;
            this.wrist = wrist;
        }

        public boolean sanity() {
            return Math.abs(bot.armMotor.getCurrentPosition() - arm) <= 800;
        }

        public boolean run() {
            bot.arm.moveArm(arm);
            bot.arm.moveWrist(wrist);

            MainTeleop.armPos = arm;
            MainTeleop.wristPos = wrist;

            return waitUntilTime() && (Math.abs(bot.armMotor.getTargetPosition() - bot.armMotor.getCurrentPosition()) <= 5);
        }
    }

    private class BasketMidFrame extends Frame {
        public double waitTime = 500;

        public boolean run() {
            bot.basket.setMiddle();

            return waitUntilTime();
        }
    }

    private class ClawStandbyFrame extends Frame {
        public double waitTime = 500;

        public boolean run() {
            bot.intake.standby();

            return waitUntilTime();
        }
    }

    private class ClawEngageFrame extends Frame {
        public double waitTime = 500;

        public boolean run() {
            bot.intake.engage();

            return waitUntilTime();
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
      new ArmWristFrame(7297, 0.279),
      new ClawEngageFrame()
    };

    public Frame[] clawToBasketFrames = new Frame[] {
      new ArmWristFrame(3362, 0.264),
      new ArmWristFrame(3362, 0.832),
      new ArmWristFrame(2862, 0.832),
      new ArmWristFrame(2359, 0.832),
      new BasketMidFrame(),
      new ClawStandbyFrame(),
      new ArmWristFrame(2359, 0.936)
    };

    public Frame[] afterGrabFrames = new Frame[] {
      new ArmWristFrame(3362, 0.264)
    };

    public Frames(Hardware bot) {
        this.bot = bot;
    }

    public void runFrames(Frame[] frames) {
        this.curFrames = frames;
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
    public void clawToBasket() { runFrames(clawToBasketFrames); }
    public void afterGrab() { runFrames(afterGrabFrames); }
}
