package org.firstinspires.ftc.teamcode;

import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.gamepad1;
import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.gamepad2;

public class ControlMap {

    // X = [] (Square)
    // Y = /_\ (Triangle)
    // B = O (Circle)
    // A = X (X)

    // MAILBOX
    public static boolean MailBoxClose = gamepad1.dpad_down || gamepad1.dpad_right;
    public static boolean MailBoxMiddle = gamepad1.left_trigger > 0.5;
    public static boolean MailBoxOpen = gamepad1.right_trigger > 0.5;


    // VIPER SLIDE
    public static boolean SlideDown = gamepad1.left_bumper;
    public static boolean SlideUp = gamepad1.right_bumper;
    public static boolean BottomBasket = gamepad1.a;
    public static boolean TopBasket = gamepad1.x;
    public static boolean BottomSlide = gamepad1.y;
    public static boolean TopPole = gamepad1.b;

    // SPEED SETTINGS
    public static boolean FastSpeed = gamepad1.left_stick_button || gamepad2.left_trigger > 0.5;
    public static boolean SlowSpeed = gamepad1.right_stick_button || gamepad2.left_bumper;

    // WRIST
    public static boolean RotateWristToMailbox = gamepad2.dpad_right;
    public static boolean RotateWristOppositeMailbox = gamepad2.dpad_up;

    // CLAW
    public static boolean OpenClaw = gamepad2.dpad_left;
    public static boolean CloseClaw = gamepad2.dpad_down;

    // ARM
    public static boolean ArmDown = gamepad2.right_trigger > 0.5;
    public static boolean ArmUp = gamepad2.right_bumper;
    public static boolean ArmPickingPosition = gamepad2.y;
    public static boolean ArmMailboxPosition = gamepad2.b;
    public static boolean ArmRestPosition = gamepad2.a;
    public static boolean ArmHangingPosition = gamepad2.x;

}
