package org.firstinspires.ftc.teamcode.htech.shooter;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.VoltageSensor;
import com.qualcomm.robotcore.hardware.configuration.typecontainers.MotorConfigurationType;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.htech.config.mechanisms_hardware.Motors;

import htech.utils.PIDController;

@Config
public class Launcher {

    DcMotorEx left;
    DcMotorEx right;
    VoltageSensor vs;

    public static double targetVelocityClose = 1985; public double currentVelocity = 0, currentVelocityRight = 0;
    public static double targetVelocityFar = 2530;
    public static double targetVelocityAuto = 1790;
    public PIDController pidController, pidControllerRight;
    public static boolean reverseMotorLeft = false;
    public static boolean reverseMotorRight = true;

    public static double kP = 0.008, kI = 0, kD = 0;
    public static double kPr = 0.02, kIr = 0, kDr = 0;
    boolean pidOn = false;
    double pidPower, pidPowerRight;
    public static double power = 1;
    public double mathVelocity = 0;
    double voltage;


    public Launcher(HardwareMap hardwareMap){

        vs = hardwareMap.voltageSensor.iterator().next();

        left = hardwareMap.get(DcMotorEx.class, Motors.launcherMotorLeft);
        left.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        left.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        left.setDirection(reverseMotorLeft ? DcMotor.Direction.REVERSE : DcMotor.Direction.FORWARD);

        MotorConfigurationType motorConfigurationTypeLeft = left.getMotorType().clone();
        motorConfigurationTypeLeft.setAchieveableMaxRPMFraction(1.0);
        left.setMotorType(motorConfigurationTypeLeft);

        right = hardwareMap.get(DcMotorEx.class, Motors.launcherMotorRight);
        right.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        right.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        right.setDirection(reverseMotorRight ? DcMotor.Direction.REVERSE : DcMotor.Direction.FORWARD);

        MotorConfigurationType motorConfigurationTypeRight = right.getMotorType().clone();
        motorConfigurationTypeRight.setAchieveableMaxRPMFraction(1.0);
        right.setMotorType(motorConfigurationTypeRight);

        pidController = new PIDController(kP, kI, kD);
        pidController.maxOutput = 1;

        pidControllerRight = new PIDController(kPr, kIr, kDr);
        pidControllerRight.maxOutput = 1;

    }

    public void setPower(double p){
        left.setPower(p);
        right.setPower(p);
    }

    public void setPowerMuie(double p){
        power = p;
//        shoot();
    }


    public void shootClose(){
        pidController.targetValue = targetVelocityClose;
        pidControllerRight.targetValue = targetVelocityClose;
        pidOn = true;
    }

    public void shootAuto(){
        pidController.targetValue = targetVelocityAuto;
        pidControllerRight.targetValue = targetVelocityAuto;
        pidOn = true;
    }

    public void shootFar(){
        pidController.targetValue = targetVelocityFar;
        pidControllerRight.targetValue = targetVelocityFar;
        pidOn = true;
    }

    public void stop(){
        pidOn = false;
        setPower(0);
    }

    int i = 10;
    public void update(){


        if(i == 10){
            voltage = vs.getVoltage();
            i = 0;
        }

        currentVelocity = left.getVelocity();
        currentVelocityRight = right.getVelocity();

        if(pidOn) {
            pidPower = pidController.update(currentVelocity);
            setPower(pidPower);


            if(pidController.p != kP) pidController.p = kP;
            if(pidController.i != kI) pidController.i = kI;
            if(pidController.d != kD) pidController.d = kD;

        }

        i++;
    }



    public boolean isAtTargetSpeed1() {
//        if(currentVelocity > pidController.targetValue - errorInVel1) return true;
//        else return false;
        if(Math.abs(currentVelocity - pidController.targetValue) < errorInVel1) return true;
        else return false;
    }

    public boolean isAtTargetSpeed2() {
//        if(currentVelocity > pidController.targetValue - errorInVel2) return true;
//        else return false;
        if(Math.abs(currentVelocity - pidController.targetValue) < errorInVel2) return true;
        else return false;
    }
    public static int errorInVel1 = 125;
    public static int errorInVel2 = 150;
}
