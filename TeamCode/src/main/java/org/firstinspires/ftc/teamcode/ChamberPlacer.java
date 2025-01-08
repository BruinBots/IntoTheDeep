package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.config.Config;

@Config
public class ChamberPlacer {
    public TeleDistanceDriver rrDriver;
    public Hardware bot;
    public ChamberState state;

    public static int startViper = 3050;
    public static int downViper = 2000;
    public static double chamberPlacerDistance = 5.5;
    public static double chamberPlacerTolerance = 0.25;

    public enum ChamberState {
        STANDBY,
        DISTANCE,
        FRAMES
    }

    public ChamberPlacer(Hardware bot, TeleDistanceDriver rrDriver) {
        this.bot = bot;
        this.rrDriver = rrDriver;
        state = ChamberState.STANDBY;
    }

    public void start() {
        state = ChamberState.DISTANCE;
    }

    public void loop() {
        if (state == ChamberState.DISTANCE) {
            rrDriver.setTarget(chamberPlacerDistance, chamberPlacerTolerance);
            if (!rrDriver.needsRunning()) {
                bot.frames.topSpecimen();
                state = ChamberState.FRAMES;
            }
        }
        else if (state == ChamberState.FRAMES) {
            if (bot.frames.isBusy()) {
                bot.frames.loop();
            }
            else {
                state = ChamberState.STANDBY;
            }
        }
    }
}
