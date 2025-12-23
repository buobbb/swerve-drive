package org.firstinspires.ftc.teamcode.htech.shooter;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.configuration.typecontainers.MotorConfigurationType;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.blob.math.PIDControllerBlob;
import org.firstinspires.ftc.teamcode.htech.config.mechanisms_hardware.Motors;

import htech.utils.PIDController;

@Config
public class TurretPidTx {

    DcMotorEx motor;
    Limelight3A ll;
    public LLResult result;
    ElapsedTime timer;
    ShooterMath math;

    public double lastPos = 0;
//
    public static double distanceThreshold = 2;



    PIDControllerBlob pidController;
    public static int targetPosClose = 2, targetPosFar = 1;

    public double error;

    public static double kP = 0.033, kI = 0, kD = 0.0015;

    public static double kF = 0.01;
    public static double ALLOWED_HEADING_ERROR = Math.toRadians(1);


    public static boolean reverseMotor = false;

    public double distance;

    public boolean readLl = false;

    public boolean far = false;
    public static double farOffset = 3;

    public TurretPidTx(HardwareMap hardwareMap){

        motor = hardwareMap.get(DcMotorEx.class, Motors.turretMotor);
        ll = hardwareMap.get(Limelight3A.class, "limelight");
        ll.pipelineSwitch(0);
        math = new ShooterMath();

        timer = new ElapsedTime();

        motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motor.setDirection(reverseMotor ? DcMotor.Direction.REVERSE : DcMotor.Direction.FORWARD);

        MotorConfigurationType motorConfigurationType = motor.getMotorType().clone();
        motorConfigurationType.setAchieveableMaxRPMFraction(1.0);
        motor.setMotorType(motorConfigurationType);

        pidController = new PIDControllerBlob(kP, kI, kD);

        ll.start();

    }

    public void setPower(double power) {
        if (Math.abs(power) < 0.1) power = 0;
        motor.setPower(power);
    }


    public void update(double voltage){

        result = ll.getLatestResult();



        if(readLl){

            if(result.isValid() && result != null){

                error = far ? Math.toRadians(targetPosFar - result.getTx()) : Math.toRadians(targetPosClose - result.getTx());
                error = math.angleToTicks(error);
                double power = pidController.calculate(0, error);
                if(error<=ALLOWED_HEADING_ERROR)
                    motor.setPower(0);
                motor.setPower(-power*voltage/12);

            }
            else{
                double power = pidController.calculate(motor.getCurrentPosition(), motor.getCurrentPosition());
                motor.setPower(power);
            }

        }
        else{
            double power = pidController.calculate(0, motor.getCurrentPosition());
            motor.setPower(power);
        }

        if(pidController.kp != kP){
            pidController.kp = kP;
        }

        if(pidController.ki != kI){
            pidController.ki = kI;
        }

        if(pidController.kd != kD){
            pidController.kd = kD;
        }
    }

}

