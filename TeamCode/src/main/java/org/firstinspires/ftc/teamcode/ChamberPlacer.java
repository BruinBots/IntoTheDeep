package org.firstinspires.ftc.teamcode;

public class ChamberPlacer {
    public TeleDistanceDriver rrDriver;
    public Hardware bot;
    public ChamberState state;

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
            rrDriver.setTarget(5.5, 0.25);
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
