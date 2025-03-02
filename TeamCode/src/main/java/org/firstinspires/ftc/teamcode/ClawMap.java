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
    // This is the HIGHEST wrist position
    public static double[] WRIST_MIN = {0.13, 0.22};

    // Maximum wrist pos (0-1)
    // This is the LOWEST wrist position
    public static double[] WRIST_MAX = {0.23, 0.32};

    // Step size of the wrist servo
    public static double[] WRIST_SPEED = {0.005, 0.005};


    // Claw open/closed positions (0-1)
    public static double[] CLAW_CLOSED_POS = {0.55, 0.65};
    public static double[] CLAW_OPENED_POS = {0.43, 0.53};

    // Enable/disable wrist controls
    public static boolean WRIST_MOVEMENT = false;

    // Reversing controls. Set to -1 per-object to reverse controls.
    public int turretFactor = 1;
    public int armFactor = 1;
    public int wristFactor = 1;

    public SBA[] sbas;
    public SBARunner runner;

    // Store current wrist position
    // INCREASING wristPos results in LOWERING the wrist
    double wristPos;

    public ClawMap(HardwareMap hardwareMap, Telemetry telemetry, Gamepad gamepad, String id) {
        // Initialize variables
        this.telemetry = telemetry;
        this.id = id;
        this.lid = Integer.parseInt(id)-1;
        this.pre = "["+id+"] ";
        this.gamepad = gamepad;

        // Check if we're using a turret servo
        if (isTurretServo) {
            // If yes, init servo
            turretServo = hardwareMap.get(Servo.class, "turretServo"+id);
        } else {
            // If not, using a motor for turret, init motor
            turretMotor = hardwareMap.get(DcMotorEx.class, "turretMotor"+id);
            turretMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        }

        // Init arm motor and set it up
        armMotor = hardwareMap.get(DcMotorEx.class, "armMotor"+id);
        armMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        armMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        // Init wrist and claw servos
        wristServo = hardwareMap.get(Servo.class, "wristServo"+id);
        clawServo = hardwareMap.get(Servo.class, "clawServo"+id);

        // Set starting wrist pos
        wristPos = WRIST_MAX[lid];

        // Set up SBA to instantly jump the arm up to the top pos between matches`
        sbas = new SBA[]{
                new MotorSBA(armMotor, 0.4, ARM_MIN[lid]),
        };
        runner = new SBARunner(); // Setup SBA runner
    }

    public void moveTurret(int move) {
        /*
        moveTurret(int move)
        Moves the turret motor/servo by a factor of move
        Higher magnitudes of move result in faster motion of the turret servo
        or larger motion of the turret motor
         */
        move = move*turretFactor; // Apply reversals if necessary
        if (move == 0) { // move == 0 indicates we don't move
            if (isTurretServo) { // If using a servo (continuous rotation), set it to the resting position
                turretServo.setPosition(TURRET_SERVO_REST_POS);
                telemetry.addData(pre+"Turret Status", "Stopping servo");
            } else { // If using a motor, set it to zero power (brake)
                turretMotor.setPower(0);
                telemetry.addData(pre+"Turret Status", "Stopping motor");
            }
            return;
        }
        // If we're actually moving the turret (move != 0)
        if (isTurretServo) { // If using servo,
            /*
            Continuous rotation servos accept values from 0-1
            They have a rest position (usually 0.5)
            Setting the servo to a position above its rest position causes it to rotate one way
            Setting it to a position below its rest position causes it to rotate the other way
            The farther the target pos is from the rest pos, the faster the servo will move
             */
            double magnitude = TURRET_SERVO_SPEED[lid]/2; // Split magnitude in half
            double target = TURRET_SERVO_REST_POS + magnitude*move; // Add the split magnitude to the rest position
            turretServo.setPosition(target);
            telemetry.addData(pre+"Turret Status", "Servo "+target);
        }
        else {
            // If using turret motor, move it in the requested direction
            turretMotor.setPower(TURRET_MOTOR_POWER);
            turretMotor.setVelocity(move*TURRET_MOTOR_SPEED, AngleUnit.DEGREES);
            turretMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            telemetry.addData(pre+"Turret Status", "Motor "+turretMotor.getCurrentPosition()+"=>"+turretMotor.getTargetPosition());
        }
    }

    public void moveArm(int move) {
        /*
        moveArm(int move)
        Moves the arm motor by a factor of move
         */
        move = move*armFactor; // Apply appropriate reversals
        double theta = (double)armMotor.getCurrentPosition() * (Math.PI/800.0); // Calculate arm angle (radians)
        double power = ARM_A + ARM_B*Math.sin(theta); // Calculate power from angle
        /*
                Arm
                 |
                 v
          ____________________________
         /\                          |
        /  \                         v
                                     mg

        When the arm approaches the horizontal position,
        more of the weight force is converted to torque
        against the motor. As a result, more motor power
        is required to hold position.
         */
        telemetry.addData("Arm Current Pos", armMotor.getCurrentPosition());
        telemetry.addData("Arm Angle", theta);
        telemetry.addData("Arm Power", power);
        if (move == 0) { // Not moving arm, just hold position
            armMotor.setPower(power);
            telemetry.addData(pre+"Arm Status", "Stopping motor");
            return;
        }
        // If moving arm
        armMotor.setPower(power);
        int curPos = armMotor.getCurrentPosition();
        int target = curPos + ARM_SPEED*move;
        // Ensure target position is within limits
        if (target >= ARM_MAX[lid]) {
            target = ARM_MAX[lid];
        } else if (target <= ARM_MIN[lid]) {
            target = ARM_MIN[lid];
        }
        armMotor.setTargetPosition(target); // Set target position
        armMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        telemetry.addData(pre+"Arm Target", "Motor "+armMotor.getCurrentPosition()+"=>"+armMotor.getTargetPosition());
    }

    public void moveWrist(int move) {
        /*

         */
        move = move*wristFactor;
        wristPos = wristPos + WRIST_SPEED[lid]*move;
        if (wristPos <= WRIST_MIN[lid]) {
            wristPos = WRIST_MIN[lid];
        } else if (wristPos >= WRIST_MAX[lid]) {
            wristPos = WRIST_MAX[lid];
        }
        wristServo.setPosition(wristPos + Math.random()*0.001);
        telemetry.addData(pre+"Wrist Status", "Servo "+wristPos);
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
        if (gamepad.dpad_right) {
            moveTurret(1);
        } else if (gamepad.dpad_left) {
            moveTurret(-1);
        } else {
            moveTurret(0);
        }

        if (gamepad.dpad_down) { // move arm up
            moveArm(1);
        } else if (gamepad.dpad_up) { // move arm down
            moveArm(-1);
        } else {
            moveArm(0);
        }

        if (WRIST_MOVEMENT) {
            if (gamepad.left_stick_y > 0) { // move wrist up
                moveWrist(1);
            } else if (gamepad.left_stick_y < 0) { // move wrist down
                moveWrist(-1);
            }
        }
        moveWrist(0);

        if (gamepad.left_bumper || gamepad.left_trigger > 0.5) { // open claw
            moveClaw(1);
        } else if (gamepad.right_bumper || gamepad.right_trigger > 0.5) { // close claw
            moveClaw(0);
        } else {
            moveClaw(-1);
        }

        if (gamepad.b && gamepad.a) {
            runner.runSBAs(sbas);
        }

        runner.loop();
    }
}
