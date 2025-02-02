package org.firstinspires.ftc.teamcode.NextYearTesting.SBAs;

import static org.firstinspires.ftc.teamcode.NextYearTesting.Utils.GlobalBot.bot;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.Arm;

import java.util.Arrays;
import java.util.Dictionary;
import java.util.Hashtable;

public class SBALexer {
    public enum Action {
        MOTOR,
        SERVO,
        WAIT,
        DISTANCE
    }

    public SBARunner runner;
    public Dictionary<String, DcMotorEx> motorMap = new Hashtable<>();
    public Dictionary<String, Servo> servoMap = new Hashtable<>();
    public Dictionary<String, Double> constants = new Hashtable<>();

    public SBALexer() {
        runner = new SBARunner();

        motorMap.put("armMotor", bot.armMotor);
        motorMap.put("viperMotorL", bot.viperMotorL);
        motorMap.put("viperMotorR", bot.viperMotorR);

        servoMap.put("wristServo", bot.wristServo);

        constants.put("clawOpenPos", 0.3);
        constants.put("clawClosedPos", 0.5);
    }

    public SBA[] scriptToSBAs(String script) {
        SBA[] sbas = new SBA[] {};
        int i = 0;
        for (String line: script.split("\n")) {
            if (line.trim().length() == 0) {
                continue;
            }
            String[] components = line.split(" ");
            Action action = Action.valueOf(components[0]);
            String[] params = Arrays.copyOfRange(components, 1, components.length);
            sbas[i] = handleAction(action, params);
            i ++;
        }
        return sbas;
    }

    public void runScript(String script) {
        runner.runSBAs(scriptToSBAs(script));
    }

    public void loop() {
        runner.loop();
    }

    public SBA handleAction(Action action, String[] params) {
        switch (action) {
            case MOTOR:
                return runMotor(params);
                break;
            case SERVO:
                return runServo(params);
                break;
            case WAIT:
                return runWait(params);
                break;
            case DISTANCE:
                return runDistance(params);
                break;
        }
        return null;
    }

    public double getParam(String param) {
        try {
            return Double.parseDouble(param);
        } catch (NumberFormatException e) {
            return constants.get(param);
        }
    }

    public SBA runMotor(String[] params) {
        String motorName = params[0];
        DcMotorEx motor = motorMap.get(motorName);
        double power;
        int target;
        if (params.length == 3) {
            power = getParam(params[1]);
            target = (int)getParam(params[2]);
        } else {
            power = Arm.ARM_POWER;
            target = (int)getParam(params[1]);
        }
        return new MotorSBA(motor, power, target);
    }

    public SBA runServo(String[] params) {
        String servoName = params[0];
        Servo servo = servoMap.get(servoName);
        double target = Double.parseDouble(params[1]);
        return new ServoSBA(servo, target);
    }

    public SBA runWait(String[] params) {
        int wait = Integer.parseInt(params[0]);
        return new WaitSBA(wait);
    }

    public SBA runDistance(String[] params) {
        double target = Double.parseDouble(params[0]);
        double tolerance = Double.parseDouble(params[1]);
        return new DistanceSBA(target, tolerance);
    }
}
