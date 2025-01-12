package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.Gamepad;

public class ControlMap {

    // X = [] (Square)
    // Y = /_\ (Triangle)
    // B = O (Circle)
    // A = X (X)

    Gamepad gamepad1;
    Gamepad gamepad2;

    public ControlMap(Gamepad gamepad1, Gamepad gamepad2) {
        this.gamepad1 = gamepad1;
        this.gamepad2 = gamepad2;
    }

    // VIPER SLIDE

    public boolean BothSlidesUp() {
        return gamepad1.dpad_up;
    }

    public boolean BothSlidesDown() {
        return gamepad1.dpad_down;
    }

    public boolean UpdateSlide() {
        return gamepad1.right_trigger > 0.5;
    }

    public boolean DistanceTest() { return gamepad1.left_trigger > 0.5; }

    public boolean RunChamberPlacer() {
        return gamepad1.a;
    }

    public boolean RunWallPicker() {
        return gamepad1.x;
    }

    public boolean BottomSlide() {
        return gamepad1.y;
    }

    public boolean TopPole() {
        return gamepad1.b;
    }

    public boolean LeftSlideUp() {
        return gamepad1.right_bumper;
    }

    public boolean LeftSlideDown() {
        return gamepad1.left_bumper;
    }

    public boolean BasketClosed() {
        return gamepad1.dpad_right;
    }

    public boolean BasketOpen() {
        return gamepad1.dpad_left;
    }

    // SPEED SETTINGS
    public boolean FastSpeed() {
        return gamepad1.left_stick_button || gamepad2.left_stick_button;
    }

    public boolean SlowSpeed() {
        return gamepad1.right_stick_button || gamepad2.right_stick_button;
    }

    // WRIST
    public boolean RotateWristToMailbox() {
        return gamepad2.dpad_down && Xtra0();
    }

    public boolean RotateWristOppositeMailbox() {
        return gamepad2.dpad_up && Xtra0();
    }

    // CLAW
    public boolean OpenClaw() {
        return gamepad2.dpad_left;
    }

    public boolean CloseClaw() {
        return gamepad2.dpad_right;
    }

    // ARM
    public boolean ArmDown() {
        return gamepad2.left_bumper;
    }

    public boolean ArmUp() {
        return gamepad2.right_bumper;
    }

    // CARJACK
    public boolean CarJackUp() {
        return gamepad2.dpad_up && Xtra1();
    }

    public boolean CarJackDown() {
        return gamepad2.dpad_down && Xtra1();
    }

    public boolean Xtra0() {
        return !(Xtra1() || Xtra2());
    }

    public boolean Xtra1() {
        return gamepad2.left_trigger > 0.5;
    }

    public boolean Xtra2() {
        return gamepad2.right_trigger > 0.5;
    }


    // MAGIC BUTTONS (Frames)
    public boolean ArmPickingPosition() {
        return gamepad2.x && Xtra0();
    }

    public boolean ArmMailboxPosition() {
        return gamepad2.b && Xtra0();
    }

    public boolean ArmRestPosition() {
        return gamepad2.a && Xtra0();
    }

    public boolean ArmPecking() {
        return gamepad2.y && Xtra0();
    }

    public boolean OpenArm() {
        return gamepad2.x && Xtra1();
    }

    public boolean CloseArm() {
        return gamepad2.b && Xtra1();
    }
}
