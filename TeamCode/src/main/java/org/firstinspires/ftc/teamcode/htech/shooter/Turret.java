package org.firstinspires.ftc.teamcode.htech.shooter;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.VoltageSensor;
import com.qualcomm.robotcore.hardware.configuration.typecontainers.MotorConfigurationType;

import org.firstinspires.ftc.teamcode.htech.config.mechanisms_hardware.Motors;
import org.firstinspires.ftc.teamcode.htech.swerve.MathUtils;
import org.firstinspires.ftc.teamcode.htech.utils.MotionProfile;

import htech.utils.PIDController;

@Config
public class Turret {

    DcMotorEx m;

    public VoltageSensor voltageSensor;

    public int targetPosition = 0, currentPosition = 0;
    public PIDController pidController;
    public static boolean reverseMotor = false;

    public static double kP = 0.07, kI = 0.0, kD = 0, kF = 0.005;
    double pidPower;

    public static boolean voltageCompensation = false;
    public static double nominalVoltage = 12.0;

    public Turret(HardwareMap hardwareMap){
        m = hardwareMap.get(DcMotorEx.class, Motors.turretMotor);

        m.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        m.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
        m.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        m.setDirection(reverseMotor ? DcMotorEx.Direction.REVERSE : DcMotorEx.Direction.FORWARD);

        MotorConfigurationType motorConfigurationType = m.getMotorType().clone();
        motorConfigurationType.setAchieveableMaxRPMFraction(1.0);
        m.setMotorType(motorConfigurationType);

        pidController = new PIDController(kP, kI, kD);
        pidController.targetValue = targetPosition;
        pidController.maxOutput = 1;
    }

    public void setVoltageSensor(VoltageSensor voltageSensor) {
        this.voltageSensor = voltageSensor;
    }

    public void setPosition(int position){
        targetPosition = position;
        pidController.targetValue = targetPosition;
    }

    public void update(){


        currentPosition = m.getCurrentPosition();

        pidPower = pidController.update(currentPosition, kF);
        m.setPower(pidPower);
        m.setPower(MathUtils.clamp(pidPower * getVoltageScale(), -1, 1));

        if(pidController.p != kP) pidController.p = kP;
        if(pidController.i != kI) pidController.i = kI;
        if(pidController.d != kD) pidController.d = kD;

    }

    private double getVoltageScale() {
        if (!voltageCompensation || voltageSensor == null) return 1;
        double voltage = voltageSensor.getVoltage();
        if (voltage <= 1e-3) return 1;
        return nominalVoltage / voltage;
    }


}