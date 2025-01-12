package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDriveCancelable;

@Config
@TeleOp(name = "Main TeleOp", group = "Iterative Opmode")
public class MainTeleop extends OpMode {
    Hardware bot;
    ControlMap controlMap;

    // drive values
    double drive = 0.0;
    double turn = 0.0;
    double strafe = 0.0;
    public static int armPos = 0;
    boolean isPixel = false;
    private FtcDashboard dash;
    private Telemetry dashTelemetry;
    public static double wristPos = 1;
    public static boolean wristArmSync = false;
    public static boolean engageAtStart = false;
    public static boolean colorActionEnabled = false;
    //    public static ColorDistanceSensor.Colors currentOP = (ColorDistanceSensor.Colors.red);
    public boolean viperLeftPressed = false;
    public boolean bothViperPressed = false;
    public boolean armPressed = false;
    public double DRIVE_SPEED;
    public double FAST_DRIVE_SPEED = 0.65; // original speed
    public double SLOW_DRIVE_SPEED = 0.30; // half fast speed

    private TeleDistanceDriver rrDriver;
    private ChamberPlacer chamberPlacer;
    private WallPicker wallPicker;

    public static boolean enableMotorBurner = false;
    MotorBurner burnerViperL;
    MotorBurner burnerViperR;
    MotorBurner burnerArm;

    @Override
    public void init() {
        dash = FtcDashboard.getInstance();
        dashTelemetry = dash.getTelemetry();
        bot = new Hardware(hardwareMap);
        controlMap = new ControlMap(gamepad1, gamepad2);
        rrDriver = new TeleDistanceDriver(hardwareMap, telemetry);
        chamberPlacer = new ChamberPlacer(bot, rrDriver);
        wallPicker = new WallPicker(bot, rrDriver);

        if (enableMotorBurner) {
            burnerViperL = new MotorBurner(bot.viperMotorL, 9, this, 2);
            burnerViperR = new MotorBurner(bot.viperMotorR, 9, this, 2);
            burnerArm = new MotorBurner(bot.armMotor, 9, this, 1);
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
    public void start() {
        bot.viper.resetEncoders();
        armPos = 0;
        wristPos = 1;
        DRIVE_SPEED = FAST_DRIVE_SPEED;

        bot.init(true);

        rrDriver.drive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        bot.frames.lift();
    }

    @Override
    public void loop() {

        rrDriver.drive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        // Drive
        drive = gamepad2.left_stick_y - gamepad1.left_stick_y;
        strafe = gamepad1.left_stick_x - gamepad2.left_stick_x;
        turn = gamepad2.right_stick_x + gamepad1.right_stick_x;

        if (drive > 1) {
            drive = 1;
        }
        if (strafe > 1) {
            strafe = 1;
        }
        if (turn > 1) {
            turn = 1;
        }

        if (drive != 0 || strafe != 0 || turn != 0) {
            rrDriver.drive.breakFollowing();
            rrDriver.setTarget(0, 0);
        }

        strafe = Math.copySign(Math.pow(strafe, 2), strafe);
        drive = Math.copySign(Math.pow(drive, 2), drive);
        turn = Math.copySign(Math.pow(turn, 2), turn);

        // Claw
        if (controlMap.OpenClaw()) {
            bot.intake.standby();
        } else if (controlMap.CloseClaw()) {
            bot.intake.engage();
        }

        // Basket
        if (controlMap.BasketOpen()) {
            bot.basket.setOpen();
        } else if (controlMap.BasketClosed()) {
            bot.basket.setClosed();
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
        } else if (wristPos < Arm.MIN_WRIST_POS) {
            wristPos = Arm.MIN_WRIST_POS;
        }

        bot.arm.loop();

        // Arm Hotkeys
        if (controlMap.ArmPickingPosition()) {
            // arm position to grab sample
            bot.frames.beforeGrab();
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
        }
        else if (controlMap.TopPole()) {
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
        if (controlMap.UpdateSlide()) {
            bot.viper.resetEncoders();
        }

        if (controlMap.RunChamberPlacer()) {
            wallPicker.stop();
            chamberPlacer.start();
        } else if (controlMap.RunWallPicker()) {
            chamberPlacer.stop();
            wallPicker.start();
        }


//        if (controlMap.SlideUp()) {
//            viperPressed = true;
//            bot.viper.moveUp(Viper.Sides.BOTH);
//        }
//        else if (controlMap.SlideDown()) {
//            viperPressed = true;
//            bot.viper.moveDown(Viper.Sides.BOTH);
//        } else {
//            if (viperPressed) {
//                viperPressed = false;
//                bot.viper.move(bot.viper.getTargetPos(Viper.Sides.BOTH), Viper.Sides.BOTH);
//            }
//        }

        if (controlMap.LeftSlideUp()) {
            viperLeftPressed = true;
            bot.viper.moveUp(Viper.Sides.LEFT);
        } else if (controlMap.LeftSlideDown()) {
            viperLeftPressed = true;
            bot.viper.moveDown(Viper.Sides.LEFT);
        } else {
            if (viperLeftPressed) {
                viperLeftPressed = false;
                bot.viper.move(Math.abs(bot.viper.getActualPosition(Viper.Sides.LEFT)), Viper.Sides.LEFT
                );
            }
        }

        if (controlMap.BothSlidesUp()) {
            bothViperPressed = true;
            bot.viper.moveUp(Viper.Sides.BOTH);
        } else if (controlMap.BothSlidesDown()){
            bothViperPressed = true;
            bot.viper.moveDown(Viper.Sides.BOTH);
        }
        else {
            if (bothViperPressed) {
                bothViperPressed = false;
                bot.viper.move(Math.abs(bot.viper.getActualPosition(Viper.Sides.LEFT)), Viper.Sides.LEFT);
                bot.viper.move(Math.abs(bot.viper.getActualPosition(Viper.Sides.RIGHT)), Viper.Sides.RIGHT);
            }
        }

        if (controlMap.CarJackUp()) {
            bot.carJack.moveUp();
        } else if (controlMap.CarJackDown()) {
            bot.carJack.moveDown();
        } else {
            bot.carJack.stop();
        }

        if (wristArmSync) {
            bot.arm.syncWristToArm();
        }

        doTelemetry("Distance", bot.distanceSensor.getValue());
        // Telemetry
        doTelemetry("Xtra0", controlMap.Xtra0());
        doTelemetry("Xtra1", controlMap.Xtra1());
        doTelemetry("Xtra2", controlMap.Xtra2());
        doTelemetry("Intake", bot.intake.getPosition());
        displayMotorTelemetry("Viper Motor L", bot.viperMotorL);
        displayMotorTelemetry("Viper Motor R", bot.viperMotorR);
        displayMotorTelemetry("Arm Motor", bot.armMotor);
        doTelemetry("Wrist Position", bot.wristServo.getPosition());
        doTelemetry("CarJack Power", bot.carJackServo.getPower());
        doTelemetry("CarJack Switch", bot.carJack.limitSwitchState());
//        doTelemetry("Color", bot.colorDistanceSensor.color);
//        doTelemetry("Distance", bot.colorDistanceSensor.READING_DISTANCE);
        doTelemetry("Is Pixel", isPixel);
        doTelemetry("Basket Pos", bot.basket.getPosition());
        telemetry.update();
        dashTelemetry.update();

        bot.arm.moveArm(armPos); // Move arm
        bot.arm.moveWrist(wristPos); // Move wrists
        bot.moveBotMecanum(drive, turn, strafe, DRIVE_SPEED); // actually move the robot

        if (enableMotorBurner) {
            burnerViperL.loop();
            burnerViperR.loop();
            burnerArm.loop();
        }

        rrDriver.drive.update();
        if (rrDriver.needsRunning() && !rrDriver.isBusy()) {
            rrDriver.loop();
        }
        chamberPlacer.loop();
        wallPicker.loop();
    }
}
