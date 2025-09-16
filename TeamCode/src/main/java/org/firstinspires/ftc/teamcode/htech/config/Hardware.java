package org.firstinspires.ftc.teamcode.htech.config;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.VoltageSensor;
import com.qualcomm.robotcore.hardware.configuration.typecontainers.MotorConfigurationType;

@Config
public class Hardware {

    public static DcMotorEx frontLeftMotor;
    public static DcMotorEx frontRightMotor;
    public static DcMotorEx backLeftMotor;
    public static DcMotorEx backRightMotor;

    public static CRServo frontLeftServo;
    public static CRServo frontRightServo;
    public static CRServo backLeftServo;
    public static CRServo backRightServo;

    public static AnalogInput frontLeftEncoder;
    public static AnalogInput frontRightEncoder;
    public static AnalogInput backLeftEncoder;
    public static AnalogInput backRightEncoder;

    public static VoltageSensor vs;

    public static boolean auto = false;

    public static void unlock(DcMotorEx motor)
    {
        MotorConfigurationType mct = motor.getMotorType();
        mct.setAchieveableMaxRPMFraction(1);
        motor.setMotorType(mct);
    }

    public static void init(HardwareMap hardwareMap){

        frontLeftMotor = hardwareMap.get(DcMotorEx.class, "m0");
        frontRightMotor = hardwareMap.get(DcMotorEx.class, "m1");
        backLeftMotor = hardwareMap.get(DcMotorEx.class, "m2");
        backRightMotor = hardwareMap.get(DcMotorEx.class, "m3");

        vs = hardwareMap.voltageSensor.iterator().next();

        unlock(frontLeftMotor);
        unlock(frontRightMotor);
        unlock(backLeftMotor);
        unlock(backRightMotor);

        frontLeftServo = hardwareMap.get(CRServo.class, "s0");
        frontRightServo = hardwareMap.get(CRServo.class, "s1");
        backLeftServo = hardwareMap.get(CRServo.class, "s2");
        backRightServo = hardwareMap.get(CRServo.class, "s3");

        frontLeftServo.setDirection(DcMotorSimple.Direction.REVERSE);
        backLeftServo.setDirection(DcMotorSimple.Direction.REVERSE);
        frontRightServo.setDirection(DcMotorSimple.Direction.REVERSE);
        backRightServo.setDirection(DcMotorSimple.Direction.REVERSE);

        frontLeftEncoder = hardwareMap.get(AnalogInput.class, "e0");
        frontRightEncoder = hardwareMap.get(AnalogInput.class, "e1");
        backLeftEncoder = hardwareMap.get(AnalogInput.class, "e2");
        backRightEncoder = hardwareMap.get(AnalogInput.class, "e3");



    }

    

}
