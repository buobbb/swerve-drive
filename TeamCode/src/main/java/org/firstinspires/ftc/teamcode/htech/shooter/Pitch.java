package org.firstinspires.ftc.teamcode.htech.shooter;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.htech.config.mechanisms_hardware.Servos;

@Config
public class Pitch {

    Servo s;

    public static double initPos = 0.5;

    public Pitch(HardwareMap hardwareMap){

        s = hardwareMap.get(Servo.class, Servos.pitchServo);
        s.setPosition(initPos);

    }

    public void setPosition(double pos){
        s.setPosition(pos);
    }

}
