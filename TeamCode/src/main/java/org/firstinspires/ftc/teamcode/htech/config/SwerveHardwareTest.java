package org.firstinspires.ftc.teamcode.htech.config;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.VoltageSensor;
import com.qualcomm.robotcore.hardware.configuration.typecontainers.MotorConfigurationType;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.blob.constants.BlobConstants;
import org.firstinspires.ftc.teamcode.blob.localization.GoBildaPinpointDriver;

@Config
public class SwerveHardwareTest {

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

    public static GoBildaPinpointDriver odo;
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
        frontRightMotor = hardwareMap.get(DcMotorEx.class, "m2e");
        backLeftMotor = hardwareMap.get(DcMotorEx.class, "m1");
        backRightMotor = hardwareMap.get(DcMotorEx.class, "m3e");

        frontLeftMotor.setDirection(DcMotorSimple.Direction.FORWARD);
        backLeftMotor.setDirection(DcMotorSimple.Direction.FORWARD);
        frontRightMotor.setDirection(DcMotorSimple.Direction.FORWARD);
        backRightMotor.setDirection(DcMotorSimple.Direction.FORWARD);

        frontLeftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        backRightMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        backLeftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        frontRightMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);

        frontLeftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        backRightMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        backLeftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        frontRightMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        frontLeftMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        backRightMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        backLeftMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        frontRightMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        vs = hardwareMap.voltageSensor.iterator().next();
        odo = hardwareMap.get(GoBildaPinpointDriver.class, BlobConstants.pinpointName);
        odo.setEncoderDirections(BlobConstants.xPodDirection, BlobConstants.yPodDirection);
        odo.setEncoderResolution(BlobConstants.podType);
        odo.setOffsets(BlobConstants.xOffset, BlobConstants.yOffset, DistanceUnit.INCH);
        odo.resetPosAndIMU();

        unlock(frontLeftMotor);
        unlock(frontRightMotor);
        unlock(backLeftMotor);
        unlock(backRightMotor);

        frontLeftServo = hardwareMap.get(CRServo.class, "s4se");
        frontRightServo = hardwareMap.get(CRServo.class, "s2se");
        backLeftServo = hardwareMap.get(CRServo.class, "s1se");
        backRightServo = hardwareMap.get(CRServo.class, "s3se");

        frontLeftServo.setDirection(DcMotorSimple.Direction.REVERSE);
        backLeftServo.setDirection(DcMotorSimple.Direction.REVERSE);
        frontRightServo.setDirection(DcMotorSimple.Direction.REVERSE);
        backRightServo.setDirection(DcMotorSimple.Direction.REVERSE);

        frontLeftEncoder = hardwareMap.get(AnalogInput.class, "a0e");
        frontRightEncoder = hardwareMap.get(AnalogInput.class, "a2e");
        backLeftEncoder = hardwareMap.get(AnalogInput.class, "a1e");
        backRightEncoder = hardwareMap.get(AnalogInput.class, "a3e");




    }



}
