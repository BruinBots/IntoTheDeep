package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;

import org.firstinspires.ftc.robotcore.external.Telemetry;

@Config
@TeleOp(name="Main TeleOp", group = "Iterative Opmode")
public class MainTeleop extends OpMode {
    Hardware bot;

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

    @Override
    public void init() {
        dash = FtcDashboard.getInstance();
        dashTelemetry = dash.getTelemetry();
        bot = new Hardware(hardwareMap);
        // reset motor encoders (remove for competition?)
        bot.armMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
//        bot.armMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        bot.viperMotorL.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
//        bot.viperMotorL.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        bot.viperMotorR.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
//        bot.viperMotorR.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        viperPos = 0;
        armPos = 0;
        wristPos = 1;

        firstLoop = true;
    }

    public void displayMotorTelemetry(String caption, DcMotorEx motor) {
        doTelemetry(caption, motor.getCurrentPosition() + "=>" + motor.getTargetPosition());
    }

    public void doTelemetry(String caption, Object obj) {
        telemetry.addData(caption, obj);
        dashTelemetry.addData(caption, obj);
    }

    @Override
    public void loop() {

        if (firstLoop) {
            // put intake servos to standby
            if (engageAtStart) {
                bot.intake.engage();
            }
            else {
                bot.intake.standby();
            }

            bot.basket.setClosed();
        }
        firstLoop = false;

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
        if (ControlMap.OpenClaw) {
            bot.intake.standby();
        } else if (ControlMap.CloseClaw) {
            bot.intake.engage();
        }

        // Arm
        if (ControlMap.ArmUp) {
            armPressed = true;
            armPos += Arm.ARM_SPEED;
        } else if (ControlMap.ArmDown) {
            armPressed = true;
            armPos -= Arm.ARM_SPEED;
        } else {
            if (armPressed) {
                armPressed = false;
                armPos = bot.armMotor.getCurrentPosition();
            }
        }

        // Wrist
        if (ControlMap.RotateWristToMailbox) {
            wristPos += Arm.WRIST_SPEED;
        } else if (ControlMap.RotateWristOppositeMailbox) {
            wristPos -= Arm.WRIST_SPEED;
        }
        bot.arm.loop();

        // Arm Hotkeys
        if (ControlMap.ArmPickingPosition) {
            // arm position to grab sample
            bot.frames.beforeGrab();
        } else if (ControlMap.ArmMailboxPosition) {
            // arm position to transfer to basket
            bot.frames.clawToBasket();
        } else if (ControlMap.ArmRestPosition) {
            // arm rest position
            bot.frames.afterGrab();
        } else if (ControlMap.ArmHangingPosition) {
            // arm hanging position
            bot.frames.peck();
        }
        bot.frames.loop();

        // Slide Hotkeys
        if (ControlMap.BottomSlide) {
            // Move slide to 0
        } else if (ControlMap.BottomBasket) {
            // Move slide to bottom basket
        } else if (ControlMap.TopBasket) {
            // Move slide to top basket
        } else if (ControlMap.TopPole) {
            // Move slide to top pole
        }

        // Speed Settings
        if (ControlMap.FastSpeed) {
            // Set Speed to Fast Speed
        } else if (ControlMap.SlowSpeed) {
            // Set Speed to Slow Speed
        }

        // Slide
        if (ControlMap.SlideUp) {
            viperPressed = true;
            viperPos += Viper.VIPER_SPEED;
        }
        else if (ControlMap.SlideDown) {
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
        if (ControlMap.MailBoxClose) {
            bot.basket.setClosed();
        } else if (ControlMap.MailBoxOpen) {
            bot.basket.setOpen();
        } else if (ControlMap.MailBoxMiddle) {
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
        bot.moveBotMecanum(drive, turn, strafe,  0.65); // actually move the robot
    }
}
