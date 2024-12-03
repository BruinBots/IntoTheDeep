package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.Gamepad;

public class ControlMap {

    // X = [] (Square)
    // Y = /_\ (Triangle)
    // B = O (Circle)
    // A = X (X)
    Gamepad gamepad1;
    Gamepad gamepad2;

    // MAILBOX
    public boolean MailBoxClose = false;
    public boolean MailBoxMiddle = false;
    public boolean MailBoxOpen = false;

    // VIPER SLIDE
    public boolean SlideDown = false;
    public boolean SlideUp = false;
    public boolean BottomBasket = false;
    public boolean TopBasket = false;
    public boolean BottomSlide = false;
    public boolean TopPole = false;

    // SPEED SETTINGS
    public boolean FastSpeed = false;
    public boolean SlowSpeed = false;

    // WRIST
    public boolean RotateWristToMailbox = false;
    public boolean RotateWristOppositeMailbox = false;

    // CLAW
    public boolean OpenClaw = false;
    public boolean CloseClaw = false;

    // ARM
    public boolean ArmDown = false;
    public boolean ArmUp = false;
    public boolean ArmPickingPosition = false;
    public boolean ArmMailboxPosition = false;
    public boolean ArmRestPosition = false;
    public boolean ArmHangingPosition = false;
    public ControlMap (Gamepad gamepad1, Gamepad gamepad2) {
        this.gamepad1 = gamepad1;
        this.gamepad2 = gamepad2;
    }

    public void updateControllers() {

        // MAILBOX
        MailBoxClose = gamepad1.dpad_down || gamepad1.dpad_right;
        MailBoxMiddle = gamepad1.left_trigger > 0.5;
        MailBoxOpen = gamepad1.right_trigger > 0.5;


        // VIPER SLIDE
        SlideDown = gamepad1.left_bumper;
        SlideUp = gamepad1.right_bumper;
        BottomBasket = gamepad1.a;
        TopBasket = gamepad1.x;
        BottomSlide = gamepad1.y;
        TopPole = gamepad1.b;

        // SPEED SETTINGS
        FastSpeed = gamepad1.left_stick_button || gamepad2.left_trigger > 0.5;
        SlowSpeed = gamepad1.right_stick_button || gamepad2.left_bumper;

        // WRIST
        RotateWristToMailbox = gamepad2.dpad_right;
        RotateWristOppositeMailbox = gamepad2.dpad_up;

        // CLAW
        OpenClaw = gamepad2.dpad_left;
        CloseClaw = gamepad2.dpad_down;

        // ARM
        ArmDown = gamepad2.right_trigger > 0.5;
        ArmUp = gamepad2.right_bumper;
        ArmPickingPosition = gamepad2.y;
        ArmMailboxPosition = gamepad2.b;
        ArmRestPosition = gamepad2.a;
        ArmHangingPosition = gamepad2.x;
    }

}
