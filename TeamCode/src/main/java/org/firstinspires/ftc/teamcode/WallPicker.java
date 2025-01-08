package org.firstinspires.ftc.teamcode;

public class WallPicker {
    public TeleDistanceDriver rrDriver;
    public Hardware bot;
    public WallState state;

    public enum WallState {
        STANDBY,
        DISTANCE,
        FRAMES
    }

    public WallPicker(Hardware bot, TeleDistanceDriver rrDriver) {
        this.bot = bot;
        this.rrDriver = rrDriver;
        state = WallState.STANDBY;
    }

    public void start() {
        state = WallState.DISTANCE;
    }

    public void loop() {
        if (state == WallState.DISTANCE) {
            rrDriver.setTarget(4, 0.4);
            if (!rrDriver.needsRunning()) {
                bot.frames.pickupFromWall();
                state = WallState.FRAMES;
            }
        }
        else if (state == WallState.FRAMES) {
            if (bot.frames.isBusy()) {
                bot.frames.loop();
            }
            else {
                state = WallState.STANDBY;
            }
        }
    }
}
