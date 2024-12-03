package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.Gamepad;

public class ControlMap {

    // X = [] (Square)
    // Y = /_\ (Triangle)
    // B = O (Circle)
    // A = X (X)
    Gamepad gamepad1;
    Gamepad gamepad2;
    public ControlMap (Gamepad gamepad1, Gamepad gamepad2) {
        this.gamepad1 = gamepad1;
        this.gamepad2 = gamepad2;
    }

    // MAILBOX
    public boolean MailBoxClose = gamepad1.dpad_down || gamepad1.dpad_right;
    public boolean MailBoxMiddle = gamepad1.left_trigger > 0.5;
    public boolean MailBoxOpen = gamepad1.right_trigger > 0.5;


    // VIPER SLIDE
    public boolean SlideDown = gamepad1.left_bumper;
    public boolean SlideUp = gamepad1.right_bumper;
    public boolean BottomBasket = gamepad1.a;
    public boolean TopBasket = gamepad1.x;
    public boolean BottomSlide = gamepad1.y;
    public boolean TopPole = gamepad1.b;

    // SPEED SETTINGS
    public boolean FastSpeed = gamepad1.left_stick_button || gamepad2.left_trigger > 0.5;
    public boolean SlowSpeed = gamepad1.right_stick_button || gamepad2.left_bumper;

    // WRIST
    public boolean RotateWristToMailbox = gamepad2.dpad_right;
    public boolean RotateWristOppositeMailbox = gamepad2.dpad_up;

    // CLAW
    public boolean OpenClaw = gamepad2.dpad_left;
    public boolean CloseClaw = gamepad2.dpad_down;

    // ARM
    public boolean ArmDown = gamepad2.right_trigger > 0.5;
    public boolean ArmUp = gamepad2.right_bumper;
    public boolean ArmPickingPosition = gamepad2.y;
    public boolean ArmMailboxPosition = gamepad2.b;
    public boolean ArmRestPosition = gamepad2.a;
    public boolean ArmHangingPosition = gamepad2.x;

}
