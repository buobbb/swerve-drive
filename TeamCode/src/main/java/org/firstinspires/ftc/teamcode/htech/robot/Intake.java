package org.firstinspires.ftc.teamcode.htech.robot;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.htech.config.mechanisms_hardware.Motors;

@Config
public class Intake {

    DcMotorEx m;
    public static boolean reverseMotor = true;

    public Intake(HardwareMap hardwareMap){

        m = hardwareMap.get(DcMotorEx.class, Motors.intakeMotor);
        m.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        m.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        m.setDirection(reverseMotor ? DcMotor.Direction.REVERSE : DcMotor.Direction.FORWARD);

    }

    public void mananca(){
        m.setPower(1);
    }

    public void stop(){
        m.setPower(0);
    }

    public void scuipa(){
        m.setPower(-1);
    }

    public void setPower(double power){
        m.setPower(power);
    }

}
