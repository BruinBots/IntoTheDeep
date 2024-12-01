package org.firstinspires.ftc.teamcode;

import static java.lang.Thread.sleep;

public class Frames {
    private class Frame {
        int arm;
        double wrist;

        public Frame(int arm, double wrist) {
            this.arm = arm;
            this.wrist = wrist;
        }

        public void run() {
            bot.arm.moveArm(arm);
            bot.arm.moveWrist(wrist);
            while (Math.abs(bot.armMotor.getCurrentPosition() - bot.armMotor.getTargetPosition()) > 5) {
                try {
                    sleep(50);
                } catch (InterruptedException e) {
                    throw new RuntimeException(e);
                }
            }
            try {
                sleep(500);
            } catch (InterruptedException e) {
                throw new RuntimeException(e);
            }
        }
    }

    private class BasketMidFrame extends Frame {
        public BasketMidFrame() {
            super(0,0);
        }

        public void run() {
            bot.basket.setMiddle();
            try {
                sleep(500);
            } catch (InterruptedException e) {
                throw new RuntimeException(e);
            }
        }
    }

    private class ClawStandbyFrame extends Frame {
        public ClawStandbyFrame() {
            super(0, 0);
        }

        public void run() {
            bot.intake.standby();
            try {
                sleep(500);
            } catch (InterruptedException e) {
                throw new RuntimeException(e);
            }
        }
    }

    private class ClawEngageFrame extends Frame {
        public ClawEngageFrame() {
            super(0, 0);
        }

        public void run() {
            bot.intake.engage();
            try {
                sleep(500);
            } catch (InterruptedException e) {
                throw new RuntimeException(e);
            }
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

    public Frame[] beforeGrabFrames = new Frame[] {
      new Frame(7297, 0.279),
      new ClawEngageFrame()
    };

    public Frame[] clawToBasketFrames = new Frame[] {
      new Frame(3362, 0.264),
      new Frame(3362, 0.832),
      new Frame(2359, 0.832),
      new BasketMidFrame(),
      new ClawStandbyFrame(),
      new Frame(2359, 0.936)
    };

    public Frame[] afterGrabFrames = new Frame[] {
      new Frame(3362, 0.264)
    };

    public Frames(Hardware bot) {
        this.bot = bot;
    }

    private void runFrames(Frame[] frames) {
        for (Frame frame: frames) {
            frame.run();
        }
    }

    public void beforeGrab() { runFrames(beforeGrabFrames); }
    public void clawToBasket() { runFrames(clawToBasketFrames); }
    public void afterGrab() { runFrames(afterGrabFrames); }
}
