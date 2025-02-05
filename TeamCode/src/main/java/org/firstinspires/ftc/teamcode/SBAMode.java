package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.NextYearTesting.SBAs.GlobalBot;
import org.firstinspires.ftc.teamcode.NextYearTesting.SBAs.MotorSBA;
import org.firstinspires.ftc.teamcode.NextYearTesting.SBAs.SBALexer;
import org.firstinspires.ftc.teamcode.NextYearTesting.SBAs.SBARunner;

@Config
@TeleOp(name="SBAMode")
public class SBAMode extends OpMode {

    public ClawMap claw;
    public SBALexer lexer;

    public static String script = "MOTOR turretMotor 500\nSERVO clawServo clawOpenPos\nWAIT 1000\nSERVO clawServo clawClosedPos";

    @Override
    public void init() {
        claw = new ClawMap(hardwareMap, telemetry, gamepad1, "1");
//        claw.init();
        GlobalBot.bot = claw;
        lexer = new SBALexer();
    }

    @Override
    public void loop() {
        if (gamepad1.a) {
            lexer.runScript(script.replace("\\n", "\n"));
        }
        lexer.loop();
        telemetry.addData("armMotor", claw.armMotor.getCurrentPosition() + "=>" + claw.armMotor.getTargetPosition());
        telemetry.addData("armPower", claw.armMotor.getPower());
        telemetry.addData("turretMotor", claw.turretMotor.getCurrentPosition() + "=>" + claw.turretMotor.getTargetPosition());
        telemetry.addData("turretPower", claw.turretMotor.getPower());
        telemetry.update();
    }
}
