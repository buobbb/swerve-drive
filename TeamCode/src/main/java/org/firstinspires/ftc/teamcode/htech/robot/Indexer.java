package org.firstinspires.ftc.teamcode.htech.robot;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.htech.config.mechanisms_hardware.Servos;

@Config
public class Indexer {

    Servo s;

    public static double initPos = 0;
    public static double retractPos = 0;
    public static double extendPos = 0;

    public Indexer(HardwareMap hardwareMap){

        s = hardwareMap.get(Servo.class, Servos.indexerServo);
        s.setPosition(initPos);

    }

    public void extend(){
        s.setPosition(extendPos);
    }

    public void retract(){
        s.setPosition(retractPos);
    }

}
