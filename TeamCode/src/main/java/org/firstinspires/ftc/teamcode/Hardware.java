package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.rev.RevColorSensorV3;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.util.Encoder;

public class Hardware {

    public RevColorSensorV3 colorSensor;
    ColorDistanceSensor colorDistanceSensor;

    // Drive motors
    public DcMotorEx leftFrontMotor;
    public DcMotorEx rightFrontMotor;
    public DcMotorEx leftBackMotor;
    public DcMotorEx rightBackMotor;

    // Arm system
    public DcMotorEx armMotor;
    public Servo wristServo;
    public Arm arm;

    // Viper slide system
    public DcMotorEx viperMotorL;
    public DcMotorEx viperMotorR;
    public Viper viper;

    // Intake system
    public Servo intakeServoNear;
    public Servo intakeServoFar;
    public Intake intake;

    public ColorDistanceSensor ColorDistanceSensor;

    public Servo basketServo;
    public Basket basket;

    public Hardware(HardwareMap hardwareMap) {

        // Color sensor
        colorSensor = hardwareMap.get(RevColorSensorV3.class, "color_sensor");
        colorDistanceSensor = new ColorDistanceSensor(colorSensor);

        // Drive motors
        leftFrontMotor = hardwareMap.get(DcMotorEx.class, "left_front");
        rightFrontMotor = hardwareMap.get(DcMotorEx.class, "right_front");
        leftBackMotor = hardwareMap.get(DcMotorEx.class, "left_back");
        rightBackMotor = hardwareMap.get(DcMotorEx.class, "right_back");

        // Reverse left motors
        leftFrontMotor.setDirection(DcMotorEx.Direction.REVERSE);
        leftBackMotor.setDirection(DcMotorEx.Direction.REVERSE);

        // ViperSlide
        viperMotorL = hardwareMap.get(DcMotorEx.class, "viper_motor_left");
        viperMotorR = hardwareMap.get(DcMotorEx.class, "viper_motor_right");
        viper = new Viper(viperMotorL, viperMotorR);

        // Arm
        armMotor = hardwareMap.get(DcMotorEx.class, "arm_motor");
        wristServo = hardwareMap.get(Servo.class, "wrist_servo");
        arm = new Arm(armMotor, wristServo);

        // Intake and Color Sensor
        ColorDistanceSensor = new ColorDistanceSensor(hardwareMap.get(RevColorSensorV3.class, "color_sensor"));
        intakeServoNear = hardwareMap.get(Servo.class, "intake_servo_near");
        intakeServoFar = hardwareMap.get(Servo.class, "intake_servo_far");
        intake = new Intake(intakeServoNear, intakeServoFar);

        basketServo = hardwareMap.get(Servo.class, "basket_servo");
        basket = new Basket(basketServo);
    }

    public void moveBotMecanum(double drive, double rotate, double strafe, double scaleFactor) {

//        drive = rampUp(drive); // use S-curve to ramp up drive gradually

        double[] wheelSpeeds = new double[4];
        wheelSpeeds[0] = drive + strafe + rotate;  // left front
        wheelSpeeds[1] = drive - strafe - rotate;  // right front
        wheelSpeeds[2] = drive - strafe + rotate;  // left rear
        wheelSpeeds[3] = drive + strafe - rotate;  // right rear

        // finding the greatest power value
        double maxMagnitude = Math.max(Math.max(Math.max(wheelSpeeds[0], wheelSpeeds[1]), wheelSpeeds[2]), wheelSpeeds[3]);

        // dividing everyone by the max power value so that ratios are same (check if sdk automatically clips to see if go build documentation works
        if (maxMagnitude > 1.0) {
            for (int i = 0; i < wheelSpeeds.length; i++) {
                wheelSpeeds[i] /= maxMagnitude;
            }
        }

        // setting motor power and scaling down to preference
        leftFrontMotor.setPower(wheelSpeeds[0] * scaleFactor);
        rightFrontMotor.setPower(wheelSpeeds[1] * scaleFactor);
        leftBackMotor.setPower(wheelSpeeds[2] * scaleFactor);
        rightBackMotor.setPower(wheelSpeeds[3] * scaleFactor);
    }
}
