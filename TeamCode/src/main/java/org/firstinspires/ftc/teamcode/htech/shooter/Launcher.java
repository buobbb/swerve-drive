package org.firstinspires.ftc.teamcode.htech.shooter;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.configuration.typecontainers.MotorConfigurationType;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.htech.config.mechanisms_hardware.Motors;

import htech.utils.PIDController;

@Config
public class Launcher {

    DcMotorEx m;

    public double targetVelocity = 0, currentVelocity = 0;
    public PIDController pidController;
    public static boolean reverseMotor = false;

    public static double kP = 0.1, kI = 0, kD = 0.0002;
    double pidPower;
    public static double power = 1;
    public double mathVelocity = 0;

    public Launcher(HardwareMap hardwareMap){
        m = hardwareMap.get(DcMotorEx.class, Motors.launcherMotor);
        m.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        m.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        m.setDirection(reverseMotor ? DcMotor.Direction.REVERSE : DcMotor.Direction.FORWARD);

        MotorConfigurationType motorConfigurationType = m.getMotorType().clone();
        motorConfigurationType.setAchieveableMaxRPMFraction(1.0);
        m.setMotorType(motorConfigurationType);

        pidController = new PIDController(kP, kI, kD);
        pidController.targetValue = targetVelocity;
        pidController.maxOutput = 1;

    }


    public void shoot(){
        targetVelocity = power;
        pidController.targetValue = targetVelocity;
    }

    public void stop(){
        targetVelocity = 0;
        pidController.targetValue = targetVelocity;
    }

    public void update(){

        currentVelocity = m.getVelocity();
        mathVelocity = m.getVelocity(AngleUnit.RADIANS);

        pidPower = pidController.update(currentVelocity);
        m.setPower(pidPower);

        if(pidController.p != kP) pidController.p = kP;
        if(pidController.i != kI) pidController.i = kI;
        if(pidController.d != kD) pidController.d = kD;

    }



}
