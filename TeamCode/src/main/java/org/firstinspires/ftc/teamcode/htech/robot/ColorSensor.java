package org.firstinspires.ftc.teamcode.htech.robot;

import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.NormalizedColorSensor;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

public class ColorSensor {

    public enum State{
        RED , BLUE , YELLOW;
    }
    public State state=State.BLUE;

    public float redError, yellowError , blueError;

    public NormalizedColorSensor distanceSensor;

    public com.qualcomm.robotcore.hardware.ColorSensor colorSensor;



    public float red=0 , blue=0 , green=0;

    public ColorSensor(HardwareMap hardwareMap){
        colorSensor = hardwareMap.get(com.qualcomm.robotcore.hardware.ColorSensor.class, "i2c1e");
        distanceSensor = hardwareMap.get(NormalizedColorSensor.class, "i2c1e");
    }






    private void updateColor()
    {

        red=colorSensor.red();
        green=colorSensor.green();
        blue=colorSensor.blue();

    }


    public float distance(float r1 , float g1  , float b1 , float r2 , float g2 , float b2)
    {
        return (float)Math.sqrt( (r1-r2)*(r1-r2) + (b1-b2)*(b1-b2) + (g1-g2)*(g1-g2));
    }

    public static float r = 200;

    private void updateState()
    {
        redError=distance(red , green , blue , 255 ,0 , 0);
        yellowError=distance(red , green , blue , 255 , 255 , 0);
        blueError=distance(red , green , blue , 0 , 0 ,255);

        if(redError<= yellowError && redError<=blueError)state=State.RED;
        if(yellowError<= redError && yellowError<=blueError)state=State.YELLOW;
        if(blueError<= yellowError && blueError<=redError)state=State.BLUE;
    }


    public void update()
    {
        updateColor();
        updateState();
    }

    public boolean hasElement() {
        return (((DistanceSensor) colorSensor).getDistance(DistanceUnit.CM) < 6 && ((DistanceSensor) colorSensor).getDistance(DistanceUnit.CM) > 5);
    }

}
