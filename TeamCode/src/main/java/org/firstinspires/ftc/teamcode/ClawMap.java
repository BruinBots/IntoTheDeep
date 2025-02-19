package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.SBAs.MotorSBA;
import org.firstinspires.ftc.teamcode.SBAs.SBA;
import org.firstinspires.ftc.teamcode.SBAs.SBARunner;

@Config
public class ClawMap {
    // Option for turret motor or servo
    public static boolean isTurretServo = true;
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
    public int lid;
    public String pre;
    public Gamepad gamepad;

    // CONSTANTS

    // Speed the turret servo moves (if servo is used)
    // USE A CONTINUOUS ROTATION SERVO
    // 0 to 1, with 0 being no movement, and 1 being
    // maximum speed
    public static double[] TURRET_SERVO_SPEED = {0.15, 0.15};

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
    public static double ARM_A = 0.3;
    public static double ARM_B = 0.3;

    // Speed of the arm motor
    // Ticks per loop
    // Higher = faster motion
    public static int ARM_SPEED = 20;

    // ARM MAX 0 MIN -180
    public static int[] ARM_MAX = {450, 450};
    public static int[] ARM_MIN = {0, 0};


    // Minimum wrist pos (0-1)
    public static double[] WRIST_MIN = {0.13, 0.13};

    // Maximum wrist pos (0-1)
    public static double[] WRIST_MAX = {0.23, 0.23};

    // Step size of the wrist servo
    public static double[] WRIST_SPEED = {0.005, 0.005};


    // Claw open/closed positions (0-1)
    public static double[] CLAW_CLOSED_POS = {0.43, 0.65};
    public static double[] CLAW_OPENED_POS = {0.55, 0.53};

    // Reversing controls. Set to -1 per-object to reverse controls.
    public int turretFactor = 1;
    public int armFactor = 1;
    public int wristFactor = 1;

    public SBA[] sbas;
    public SBARunner runner;

    public ClawMap(HardwareMap hardwareMap, Telemetry telemetry, Gamepad gamepad, String id) {
        this.telemetry = telemetry;
        this.id = id;
        this.lid = Integer.parseInt(id)-1;
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
        armMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        wristServo = hardwareMap.get(Servo.class, "wristServo"+id);
        clawServo = hardwareMap.get(Servo.class, "clawServo"+id);

        sbas = new SBA[]{
                new MotorSBA(armMotor, 0.6, ARM_MIN[lid]),
        };
        runner = new SBARunner();
    }

    public void moveTurret(int move) {
        move = move*turretFactor;
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
            double magnitude = TURRET_SERVO_SPEED[lid]/2;
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
        move = move*armFactor;
        double theta = (double)armMotor.getCurrentPosition() * (Math.PI/800.0);
        double power = ARM_A + ARM_B*Math.sin(theta);
        telemetry.addData("Arm Current Pos", armMotor.getCurrentPosition());
        telemetry.addData("Arm Angle", theta);
        telemetry.addData("Arm Power", power);
        if (move == 0) {
            armMotor.setPower(power);
            telemetry.addData(pre+"Arm Status", "Stopping motor");
            return;
        }
        armMotor.setPower(power);
        int curPos = armMotor.getCurrentPosition();
        int target = curPos + ARM_SPEED*move;
        if (target >= ARM_MAX[lid]) {
            target = ARM_MAX[lid];
        } else if (target <= ARM_MIN[lid]) {
            target = ARM_MIN[lid];
        }
        armMotor.setTargetPosition(target);
        armMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        telemetry.addData(pre+"Arm Target", "Motor "+armMotor.getCurrentPosition()+"=>"+armMotor.getTargetPosition());
    }

    public void moveWrist(int move) {
        move = move*wristFactor;
        double target = wristServo.getPosition() + WRIST_SPEED[lid]*move;
        if (target <= WRIST_MIN[lid]) {
            target = WRIST_MIN[lid];
        } else if (target >= WRIST_MAX[lid]) {
            target = WRIST_MAX[lid];
        }
        wristServo.setPosition(target);
        telemetry.addData(pre+"Wrist Status", "Servo "+target);
    }

    public void moveClaw(int move) {
        if (move == 1) {
            clawServo.setPosition(CLAW_CLOSED_POS[lid]);
            telemetry.addData(pre+"Claw Status", "Closed "+CLAW_CLOSED_POS[lid]);
        } else if (move == 0) {
            clawServo.setPosition(CLAW_OPENED_POS[lid]);
            telemetry.addData(pre+"Claw Status", "Open "+CLAW_OPENED_POS[lid]);
        } else {
            if (clawServo.getPosition() == CLAW_OPENED_POS[lid]) {
                telemetry.addData(pre+"Claw Status", "Open "+CLAW_OPENED_POS[lid]);
            } else {
                telemetry.addData(pre+"Claw Status", "Closed "+CLAW_CLOSED_POS[lid]);
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
        if (gamepad.right_stick_x > 0.8) {
            moveTurret(1);
        } else if (gamepad.right_stick_x < -0.8) {
            moveTurret(-1);
        } else {
            moveTurret(0);
        }

        if (gamepad.right_stick_y > 0) {
            moveArm(1);
        } else if (gamepad.right_stick_y < 0) {
            moveArm(-1);
        } else {
            moveArm(0);
        }

        if (gamepad.left_stick_y > 0) {
            moveWrist(1);
        } else if (gamepad.left_stick_y < 0) {
            moveWrist(-1);
        } else {
            moveWrist(0);
        }

        if (gamepad.left_bumper) {
            moveClaw(1);
        } else if (gamepad.right_bumper) {
            moveClaw(0);
        } else {
            moveClaw(-1);
        }

        if (gamepad.dpad_up && gamepad.a) {
            runner.runSBAs(sbas);
        }

        runner.loop();
    }
}
