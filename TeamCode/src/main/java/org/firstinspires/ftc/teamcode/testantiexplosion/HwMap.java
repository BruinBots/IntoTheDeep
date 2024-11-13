package org.firstinspires.ftc.teamcode.testantiexplosion;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class HwMap {

    public DcMotorEx motor;

    public HwMap (HardwareMap hardwareMap) {
        motor = hardwareMap.get(DcMotorEx.class, "motor");
    }

}
