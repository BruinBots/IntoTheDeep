package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;

import org.firstinspires.ftc.robotcore.external.Telemetry;

@Config
@TeleOp(name="Main TeleOp", group = "Iterative Opmode")
public class MainTeleop extends OpMode {
    Hardware bot;
    ControlMap controlMap;

    // drive values
    double drive = 0.0;
    double turn = 0.0;
    double strafe = 0.0;

    public static int viperPos = 0;
    public static int armPos = 0;

    boolean isPixel = false;
    private FtcDashboard dash;
    private Telemetry dashTelemetry;

    public static double wristPos = 1;
    public static boolean wristArmSync = false;
    public static boolean engageAtStart = false;
    public static boolean colorActionEnabled = false;
    public static ColorDistanceSensor.Colors currentOP = (ColorDistanceSensor.Colors.red);
    public boolean viperPressed = false;
    public boolean armPressed = false;
    public boolean firstLoop = true;

    public double DRIVE_SPEED;
    public double FAST_DRIVE_SPEED = 0.65; // original speed
    public double SLOW_DRIVE_SPEED = 0.30; // half fast speed

    @Override
    public void init() {
        dash = FtcDashboard.getInstance();
        dashTelemetry = dash.getTelemetry();
        bot = new Hardware(hardwareMap);
        controlMap = new ControlMap(gamepad1, gamepad2);
    }

    public void displayMotorTelemetry(String caption, DcMotorEx motor) {
        doTelemetry(caption, motor.getCurrentPosition() + "=>" + motor.getTargetPosition());
    }

    public void doTelemetry(String caption, Object obj) {
        telemetry.addData(caption, obj);
        dashTelemetry.addData(caption, obj);
    }

    @Override
    public void start() {

        viperPos = 0;
        armPos = 0;
        wristPos = 1;
        DRIVE_SPEED = FAST_DRIVE_SPEED;

        bot.init(true);


        bot.frames.lift();
    }

    @Override
    public void loop() {

        // Drive
        drive = gamepad2.left_stick_y - gamepad1.left_stick_y;
        strafe = gamepad1.left_stick_x - gamepad2.left_stick_x;
        turn = gamepad2.right_stick_x + gamepad1.right_stick_x;

        if (drive > 1) { drive = 1; }
        if (strafe > 1) { strafe = 1; }
        if (turn > 1) { turn = 1; }

        strafe = Math.copySign(Math.pow(strafe, 2), strafe);
        drive = Math.copySign(Math.pow(drive, 2), drive);
        turn = Math.copySign(Math.pow(turn, 2), turn);

        // Claw
        if (controlMap.OpenClaw()) {
            bot.intake.standby();
        } else if (controlMap.CloseClaw()) {
            bot.intake.engage();
        }

        if (controlMap.UpdateSlide()) {
            bot.viperMotorL.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            bot.viperMotorR.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            viperPos = 0;
        }

        // Arm
        if (controlMap.ArmUp()) {
            armPressed = true;
            armPos += Arm.ARM_SPEED;
        } else if (controlMap.ArmDown()) {
            armPressed = true;
            armPos -= Arm.ARM_SPEED;
        } else {
            if (armPressed) {
                armPressed = false;
                armPos = bot.armMotor.getCurrentPosition();
            }
        }

        // Wrist
        if (controlMap.RotateWristToMailbox()) {
            wristPos += Arm.WRIST_SPEED;
        } else if (controlMap.RotateWristOppositeMailbox()) {
            wristPos -= Arm.WRIST_SPEED;
        }

        if (wristPos > Arm.MAX_WRIST_POS) {
            wristPos = Arm.MAX_WRIST_POS;
        }
        else if (wristPos < Arm.MIN_WRIST_POS) {
            wristPos = Arm.MIN_WRIST_POS;
        }

        bot.arm.loop();

        // Arm Hotkeys
        if (controlMap.ArmPickingPosition()) {
            // arm position to grab sample
            bot.frames.beforeGrab();
        } else if (controlMap.ArmMailboxPosition()) {
            // arm position to transfer to basket
            bot.frames.clawToBasket();
        } else if (controlMap.ArmRestPosition()) {
            // arm rest position
            bot.frames.afterGrab();
        } else if (controlMap.ArmPecking()) {
            // arm hanging position
            bot.frames.peck();
        }
        bot.frames.loop();

        // Slide Hotkeys
        if (controlMap.BottomSlide()) {
            // Move slide to 0
            bot.frames.zeroBasket();
        } else if (controlMap.BottomBasket()) {
            // Move slide to bottom basket
            bot.frames.bottomBasket();
        } else if (controlMap.TopBasket()) {
            // Move slide to top basket
            bot.frames.topBasket();
        } else if (controlMap.TopPole()) {
            // Move slide to top pole
            bot.frames.topPole();
        }

        if (controlMap.OpenArm()) {
            bot.frames.uncurlArm();
        } else if (controlMap.CloseArm()) {
            bot.frames.curlArm();
        }

        // Speed Settings
        if (controlMap.FastSpeed()) {
            // Set Speed to Fast Speed
            DRIVE_SPEED = FAST_DRIVE_SPEED;
        } else if (controlMap.SlowSpeed()) {
            // Set Speed to Slow Speed
            DRIVE_SPEED = SLOW_DRIVE_SPEED;
        }

        // Slide
        if (controlMap.SlideUp()) {
            viperPressed = true;
            viperPos += Viper.VIPER_SPEED;
        }
        else if (controlMap.SlideDown()) {
            viperPressed = true;
            viperPos -= Viper.VIPER_SPEED;
        } else {
            if (viperPressed) {
                viperPressed = false;
                viperPos = bot.viper.getActualPos();
            }
        }
        bot.viper.loop();

        // Mailbox
        if (controlMap.MailBoxClose()) {
            bot.basket.setClosed();
        } else if (controlMap.MailBoxOpen()) {
            bot.basket.setOpen();
        } else if (controlMap.MailBoxMiddle()) {
            bot.basket.setMiddle();
        }

        if (wristArmSync) {
            bot.arm.syncWristToArm();
        }

        //        bot.colorDistanceSensor.loop();
//        if (bot.colorDistanceSensor.READING_DISTANCE <= 1) {
//            isPixel = true;
//        }
//        else {
//            isPixel = false;
//        }
//
//        if (isPixel && colorActionEnabled) {
//            // If the pixel is not the current OP color, outtake for 1 second
//            ColorDistanceSensor.Colors color = bot.colorDistanceSensor.color;
//            if (color != currentOP && color != ColorDistanceSensor.Colors.yellow) {
//                bot.intake.standby();
//            } else {
//                bot.intake.engage();
//            }
//        }

        // Telemetry
        doTelemetry("Xtra0", controlMap.Xtra0());
        doTelemetry("Xtra1", controlMap.Xtra1());
        doTelemetry("Xtra2", controlMap.Xtra2());
        doTelemetry("Intake", bot.intake.getPos());
        displayMotorTelemetry("Viper Motor L", bot.viperMotorL);
        displayMotorTelemetry("Viper Motor R", bot.viperMotorR);
        displayMotorTelemetry("Arm Motor", bot.armMotor);
        doTelemetry("Wrist Position", bot.wristServo.getPosition());
//        doTelemetry("Color", bot.colorDistanceSensor.color);
//        doTelemetry("Distance", bot.colorDistanceSensor.READING_DISTANCE);
        doTelemetry("Is Pixel", isPixel);
        doTelemetry("Basket Pos", bot.basket.getPosition());
        telemetry.update();
        dashTelemetry.update();

        bot.arm.moveArm(armPos); // Move arm
        bot.arm.moveWrist(wristPos); // Move wrists
        bot.viper.move(viperPos); // Move viper
        bot.moveBotMecanum(drive, turn, strafe,  DRIVE_SPEED); // actually move the robot
    }
}
