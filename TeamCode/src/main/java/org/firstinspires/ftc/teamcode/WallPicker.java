package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.config.Config;

@Config
public class WallPicker {
    public TeleDistanceDriver rrDriver;
    public Hardware bot;
    public WallState state;

    public static int wallPickerPos = 180;
    public static int afterWallPickerPos = ChamberPlacer.startViper;
    public static double wallPickerDistance = 3.75;
    public static double wallPickerTolerance = 0.25;

    public enum WallState {
        PRE_FRAMES,
        STANDBY,
        DISTANCE,
        FRAMES
    }

    public WallPicker(Hardware bot, TeleDistanceDriver rrDriver) {
        this.bot = bot;
        this.rrDriver = rrDriver;
        state = WallState.STANDBY;
    }

    public void stop() { state = WallPicker.WallState.STANDBY; }

    public void start() {
        bot.frames.beforeWall();
        state = WallState.PRE_FRAMES;
    }

    public void loop() {
        if (state == WallState.PRE_FRAMES) {
            if (bot.frames.isBusy()) {
                bot.frames.loop();
            }
            else {
                state = WallState.DISTANCE;
            }
        }
        else if (state == WallState.DISTANCE) {
            rrDriver.setTarget(wallPickerDistance, wallPickerTolerance);
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
