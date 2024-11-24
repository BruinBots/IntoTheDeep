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

    int viperPos = 0;
    int armPos = 0;

    boolean isPixel = false;
    private FtcDashboard dash;
    private Telemetry dashTelemetry;

    public static double wristPos = 1;
    public static boolean wristArmSync = false;
    public static boolean engageAtStart = false;
    public static boolean colorActionEnabled = false;
    public static ColorDistanceSensor.Colors currentOP = (ColorDistanceSensor.Colors.red);

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
        } else if (gamepad2.right_bumper) {
            armPos += Arm.ARM_SPEED;
        } else if (gamepad2.right_trigger > 0.5) {
            armPos -= Arm.ARM_SPEED;
        }

        if (gamepad2.dpad_up) {
            wristPos += Arm.WRIST_SPEED;
        } else if (gamepad2.dpad_right) {
            wristPos -= Arm.WRIST_SPEED;
        }
        bot.arm.loop();

        if (gamepad2.y) {
            // arm position to grab sample
        } else if (gamepad2.b) {
            // arm position to transfer to basket
        } else if (gamepad2.a) {
            // arm rest position
        } else if (gamepad2.x) {
            // arm hanging position
        }

        // 720 low bar for hang

        // Viper
        if (gamepad1.right_bumper) {
            viperPos += Viper.VIPER_SPEED;
        }
        else if (gamepad1.right_trigger > 0.5) {
            viperPos -= Viper.VIPER_SPEED;
        } else {
            viperPos = bot.viper.getActualPos();
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

        // Telemetry
        telemetry.addData("Near Servo", bot.intake.getNearPos());
        telemetry.addData("Far Servo", bot.intake.getFarPos());
        displayMotorTelemetry("Viper Motor L", bot.viperMotorL);
        displayMotorTelemetry("Viper Motor R", bot.viperMotorR);
        displayMotorTelemetry("Arm Motor", bot.armMotor);
        telemetry.addData("Wrist Position", bot.wristServo.getPosition());
//        telemetry.addData("Color", bot.colorDistanceSensor.color);
//        telemetry.addData("Distance", bot.colorDistanceSensor.READING_DISTANCE);
        telemetry.addData("Is Pixel", isPixel);
        telemetry.update();
        dashTelemetry.update();

        bot.arm.moveArm(armPos); // Move arm
        bot.arm.moveWrist(wristPos); // Move wrists
        bot.viper.move(viperPos); // Move viper
        bot.moveBotMecanum(drive, turn, strafe,  0.65); // actually move the robot
    }
}
