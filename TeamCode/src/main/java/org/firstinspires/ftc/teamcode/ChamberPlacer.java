package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.config.Config;

@Config
public class ChamberPlacer {
    public TeleDistanceDriver rrDriver;
    public Hardware bot;
    public ChamberState state;

    public static int startViper = 3050;
    public static int downViper = 2000;
    public static double chamberPlacerDistance = 5.25;
    public static double chamberPlacerTolerance = 0.25;

    public enum ChamberState {
        PRE_FRAMES,
        STANDBY,
        DISTANCE,
        POST_FRAMES
    }

    public ChamberPlacer(Hardware bot, TeleDistanceDriver rrDriver) {
        this.bot = bot;
        this.rrDriver = rrDriver;
        state = ChamberState.STANDBY;
    }

    public void stop() {
        state = ChamberState.STANDBY;
    }

    public void start() {
        bot.frames.beforeChamber();
        state = ChamberState.PRE_FRAMES;
    }

    public void loop() {
        if (state == ChamberState.PRE_FRAMES) {
            if (bot.frames.isBusy()) {
                bot.frames.loop();
            }
            else {
                state = ChamberState.DISTANCE;
            }
        }
        else if (state == ChamberState.DISTANCE) {
            rrDriver.setTarget(chamberPlacerDistance, chamberPlacerTolerance);
            if (!rrDriver.needsRunning()) {
                bot.frames.topSpecimen();
                state = ChamberState.POST_FRAMES;
            }
        }
        else if (state == ChamberState.POST_FRAMES) {
            if (bot.frames.isBusy()) {
                bot.frames.loop();

            }
            else {
                state = ChamberState.STANDBY;
            }
        }
    }
}
