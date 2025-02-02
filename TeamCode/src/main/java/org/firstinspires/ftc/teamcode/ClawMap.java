package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

@Config
public class ClawMap {
    // Option for turret motor or servo
    public static boolean isTurretServo = false;
    public DcMotorEx turretMotor;
    public Servo turretServo;

    // arm motor and wrist servo
    public DcMotorEx armMotor;
    public Servo wristServo;

    // claw servo
    public Servo clawServo;

    // Telemetry
    public Telemetry telemetry;
    public String id;
    public String pre;
    public Gamepad gamepad;

    // CONSTANTS

    // Speed the turret servo moves (if servo is used)
    // USE A CONTINUOUS ROTATION SERVO
    // 0 to 1, with 0 being no movement, and 1 being
    // maximum speed
    public static double TURRET_SERVO_SPEED = 0.5;

    // Position the turret servo is at rest (usually 0.5)
    public static double TURRET_SERVO_REST_POS = 0.5;

    // Speed the turret motor moves (if motor is used)
    // Ticks per loop
    // Higher = larger steps
    public static double TURRET_MOTOR_SPEED = 30;

    // Power of the turret motor
    // Higher = faster motion
    public static double TURRET_MOTOR_POWER = 0.2;


    // Power of the arm motor
    // Higher = larger steps
    public static double ARM_POWER = 0.5;

    // Speed of the arm motor
    // Ticks per loop
    // Higher = faster motion
    public static int ARM_SPEED = 10;


    // Minimum wrist pos (0-1)
    public static double WRIST_MIN = 0;

    // Maximum wrist pos (0-1)
    public static double WRIST_MAX = 1;

    // Step size of the wrist servo
    public static double WRIST_SPEED = 0.005;


    // Claw open/closed positions (0-1)
    public static double CLAW_CLOSED_POS = 0.5;
    public static double CLAW_OPENED_POS = 0.75;

    public ClawMap(HardwareMap hardwareMap, Telemetry telemetry, Gamepad gamepad, String id) {
        this.telemetry = telemetry;
        this.id = id;
        this.pre = "["+id+"] ";
        this.gamepad = gamepad;

        if (isTurretServo) {
            turretServo = hardwareMap.get(Servo.class, "turretServo"+id);
        } else {
            turretMotor = hardwareMap.get(DcMotorEx.class, "turretMotor"+id);
            turretMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        }

        armMotor = hardwareMap.get(DcMotorEx.class, "armMotor"+id);
        armMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        wristServo = hardwareMap.get(Servo.class, "wristServo"+id);
        clawServo = hardwareMap.get(Servo.class, "clawServo"+id);
    }

    public void moveTurret(int move) {
        if (move == 0) {
            if (isTurretServo) {
                turretServo.setPosition(TURRET_SERVO_REST_POS);
                telemetry.addData(pre+"Turret Status", "Stopping servo");
            } else {
                turretMotor.setPower(0);
                telemetry.addData(pre+"Turret Status", "Stopping motor");
            }
            return;
        }
        if (isTurretServo) {
            double magnitude = TURRET_SERVO_SPEED/2;
            double target = TURRET_SERVO_REST_POS + magnitude*move;
            turretServo.setPosition(target);
            telemetry.addData(pre+"Turret Status", "Servo "+target);
        }
        else {
            turretMotor.setPower(TURRET_MOTOR_POWER);
            turretMotor.setVelocity(move*TURRET_MOTOR_SPEED, AngleUnit.DEGREES);
            turretMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            telemetry.addData(pre+"Turret Status", "Motor "+turretMotor.getCurrentPosition()+"=>"+turretMotor.getTargetPosition());
        }
    }

    public void moveArm(int move) {
        if (move == 0) {
            armMotor.setPower(0);
            telemetry.addData(pre+"Arm Status", "Stopping motor");
            return;
        }
        armMotor.setPower(ARM_POWER);
        armMotor.setTargetPosition(armMotor.getCurrentPosition() + ARM_SPEED*move);
        armMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        telemetry.addData(pre+"Arm Status", "Motor "+armMotor.getCurrentPosition()+"=>"+armMotor.getTargetPosition());
    }

    public void moveWrist(int move) {
        double target = wristServo.getPosition() + WRIST_SPEED*move;
        if (target < WRIST_MIN) {
            target = WRIST_MIN;
        } else if (target > WRIST_MAX) {
            target = WRIST_MAX;
        }
        wristServo.setPosition(target);
        telemetry.addData(pre+"Wrist Status", "Servo "+target);
    }

    public void moveClaw(int move) {
        if (move == 1) {
            clawServo.setPosition(CLAW_CLOSED_POS);
            telemetry.addData(pre+"Claw Status", "Closed "+CLAW_CLOSED_POS);
        } else if (move == 0) {
            clawServo.setPosition(CLAW_OPENED_POS);
            telemetry.addData(pre+"Claw Status", "Open "+CLAW_OPENED_POS);
        } else {
            if (clawServo.getPosition() == CLAW_OPENED_POS) {
                telemetry.addData(pre+"Claw Status", "Open "+CLAW_OPENED_POS);
            } else {
                telemetry.addData(pre+"Claw Status", "Closed "+CLAW_CLOSED_POS);
            }
        }
    }

    public void init() {
        moveTurret(0);
        moveArm(0);
        moveWrist(0);
        moveClaw(0);
    }

    public void loop() {
        if (gamepad.left_stick_x > 0) {
            moveTurret(1);
        } else if (gamepad.left_stick_x < 0) {
            moveTurret(-1);
        } else {
            moveTurret(0);
        }

        if (gamepad.left_stick_y > 0) {
            moveArm(1);
        } else if (gamepad.left_stick_y < 0) {
            moveArm(-1);
        } else {
            moveArm(0);
        }

        if (gamepad.dpad_up) {
            moveWrist(1);
        } else if (gamepad.dpad_down) {
            moveWrist(-1);
        } else {
            moveWrist(0);
        }

        if (gamepad.dpad_right) {
            moveClaw(1);
        } else if (gamepad.dpad_left) {
            moveClaw(0);
        } else {
            moveClaw(-1);
        }
    }
}
