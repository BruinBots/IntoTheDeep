package org.firstinspires.ftc.teamcode.NextYearTesting.SBAs;

import static org.firstinspires.ftc.teamcode.NextYearTesting.SBAs.GlobalBot.bot;

import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.ClawMap;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.Dictionary;
import java.util.Hashtable;

public class SBALexer {
    public enum Action {
        MOTOR,
        SERVO,
        WAIT,
    }

    public SBARunner runner;

    // Dictionary maps (populated in initializer)
    public Dictionary<String, DcMotorEx> motorMap = new Hashtable<>();
    public Dictionary<String, Double> motorPowers = new Hashtable<>();
    public Dictionary<String, Servo> servoMap = new Hashtable<>();
    public Dictionary<String, Double> constants = new Hashtable<>();

    /*
    EXAMPLE SCRIPT:

    ARM wallPickerArmPos
    DISTANCE wallPickerDistance wallPickerTolerance
    SERVO basketServo baksetOpenPos
    WAIT 250
    ARM wallPickerAfterArmPos

    This should be identical to the existing WallPicker code.
     */

    public SBALexer() {
        runner = new SBARunner();

        // Populate motor dictionary
        motorMap.put("armMotor", bot.armMotor);
        motorMap.put("turretMotor", bot.turretMotor);
        // TODO: Figure out how to handle two motors at once (maybe a 2MOTOR operation)

        // Default motor powers
        motorPowers.put("armMotor", 0.5);
        motorPowers.put("turretMotor", 0.2);

        // Populate servo dictionary
        servoMap.put("wristServo", bot.wristServo);
        servoMap.put("clawServo", bot.clawServo);

        // Populate constants
        constants.put("clawOpenPos", ClawMap.CLAW_OPENED_POS);
        constants.put("clawClosedPos", ClawMap.CLAW_CLOSED_POS);
    }

    public SBA[] scriptToSBAs(String script) {
        ArrayList<SBA> sbas = new ArrayList<>();
        int i = 0;
        // Iterate through lines of the script
        for (String line: script.split("\n")) {
            if (line.trim().length() == 0 || line.startsWith("//")) { // Ensure line isn't empty and isn't a comment
                continue;
            }
            String[] components = line.split(" "); // Split line by spaces into components
            Action action = Action.valueOf(components[0]); // Action is the first component
            String[] params = Arrays.copyOfRange(components, 1, components.length); // Parameters are everything after the action
            sbas.add(handleAction(action, params)); // Convert the action + params to an SBA
            i ++;
        }
        return sbas.toArray(new SBA[0]);
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
            case SERVO:
                return runServo(params);
            case WAIT:
                return runWait(params);
        }
        return null;
    }

    public double getParam(String param) {
        try {
            // If the parameter is a double, return it as a double.
            return Double.parseDouble(param);
        } catch (NumberFormatException e) {
            // If the parameter is a string (a constant's name), get the value of the constant and return it.
            return constants.get(param);
        }
    }

    public SBA runMotor(String[] params) {
        // Fetch motor information
        String motorName = params[0];
        DcMotorEx motor = motorMap.get(motorName);

        double power;
        int target;
        /* 3-parameter MOTOR commands are in the form (extra spaces added for labels):
        MOTOR armMotor      0.5     4400
              ^             ^       ^
              motor name    power   target position
         */
        if (params.length == 3) {
            power = getParam(params[1]);
            target = (int)getParam(params[2]);
        }
        /* 2-parameter MOTOR commands are in the form (extra spaces added for labels):
        MOTOR armMotor      4400
              ^             ^
              motor name    target position

        power = default power for that motor
         */
        else {
            power = motorPowers.get(motorName);
            target = (int)getParam(params[1]);
        }

        return new MotorSBA(motor, power, target);
    }

    public SBA runServo(String[] params) {
        // Fetch servo information
        String servoName = params[0];
        Servo servo = servoMap.get(servoName);
        // Get target position
        double target = getParam(params[1]);

        return new ServoSBA(servo, target);
    }

    public SBA runWait(String[] params) {
        // Get wait time (ms)
        int wait = Integer.parseInt(params[0]);

        return new WaitSBA(wait);
    }
}
