package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

public class Hardware {



    public static DcMotorEx m0, m1, m2e, m3e;
    public static Servo s0se, s1se, s2se, s3se, s4se, s5se;
    public static AnalogInput a0e, a1e, a2e, a3e;

    public static void init(HardwareMap hardwareMap)
    {
        m0 =hardwareMap.get(DcMotorEx.class, "m0");
        m1 =hardwareMap.get(DcMotorEx.class, "m1");
        m2e =hardwareMap.get(DcMotorEx.class, "m2e");
        m3e =hardwareMap.get(DcMotorEx.class, "m3e");

        s0se =hardwareMap.get(Servo.class, "s0se");
        s1se =hardwareMap.get(Servo.class, "s1se");
        s2se =hardwareMap.get(Servo.class, "s2se");
        s3se =hardwareMap.get(Servo.class, "s3se");
        s4se =hardwareMap.get(Servo.class, "s4se");
        s5se =hardwareMap.get(Servo.class, "s5se");

        a0e =hardwareMap.get(AnalogInput.class, "a0e");
        a1e =hardwareMap.get(AnalogInput.class, "a1e");
        a2e =hardwareMap.get(AnalogInput.class, "a2e");
        a3e =hardwareMap.get(AnalogInput.class, "a3e");


    }
}
