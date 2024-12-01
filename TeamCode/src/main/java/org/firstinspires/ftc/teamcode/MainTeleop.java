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

        // put intake servos to standby
        if (engageAtStart) {
            bot.intake.engage();
        }
        else {
            bot.intake.standby();
        }

        bot.basket.setClosed();

        viperPos = 0;
        armPos = 0;
        wristPos = 1;
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

        // Arm
        if (gamepad2.dpad_down) {
            bot.intake.standby();
        }

        // Wrist
        if (gamepad2.dpad_left) {
            bot.intake.engage();
        }

        // Arm
        doTelemetry("RightBumper", gamepad2.right_bumper);
        doTelemetry("RightTrigger", gamepad2.right_trigger);
        if (gamepad2.right_bumper) {
            telemetry.addLine("Arm +");
            armPressed = true;
            armPos += Arm.ARM_SPEED;
        } else if (gamepad2.right_trigger > 0.5) {
            telemetry.addLine("Arm -");
            armPressed = true;
            armPos -= Arm.ARM_SPEED;
        } else {
            if (armPressed) {
                armPressed = false;
                armPos = bot.armMotor.getCurrentPosition();
            }
        }

        if (gamepad2.dpad_up) {
            wristPos += Arm.WRIST_SPEED;
        } else if (gamepad2.dpad_right) {
            wristPos -= Arm.WRIST_SPEED;
        }
        bot.arm.loop();

        if (gamepad2.y) {
            // arm position to grab sample
            bot.frames.beforeGrab();
        } else if (gamepad2.b) {
            // arm position to transfer to basket
            bot.frames.clawToBasket();
        } else if (gamepad2.a) {
            // arm rest position
            bot.frames.afterGrab();
        } else if (gamepad2.x) {
            // arm hanging position
        }

        bot.frames.loop();

        // 720 low bar for hang

        // Viper
        if (gamepad1.right_bumper) {
            viperPressed = true;
            viperPos += Viper.VIPER_SPEED;
        }
        else if (gamepad1.right_trigger > 0.5) {
            viperPressed = true;
            viperPos -= Viper.VIPER_SPEED;
        } else {
            if (viperPressed) {
                viperPressed = false;
                viperPos = bot.viper.getActualPos();
            }
        }
        bot.viper.loop();

        if (gamepad1.dpad_down) {
            bot.basket.setClosed();
        } else if (gamepad1.dpad_right) {
            bot.basket.setOpen();
        } else if (gamepad1.dpad_left) {
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
