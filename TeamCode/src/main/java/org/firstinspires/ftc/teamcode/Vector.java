package org.firstinspires.ftc.teamcode;

import java.lang.Math;

public class Vector {

    double x, y;
    double multiplier;
    double minus;
    public Vector(double x, double y)
    {
        this.x=x;
        this.y=y;
    }

    public double getDistance()
    {
        return Math.sqrt(x*x + y*y);
    }

    public double getAngle()
    {
        if(y>0 && x>=0){multiplier=0; minus=1;}
        if(x>0 && y<=0){multiplier=Math.PI; minus=-1;}
        if(x<=0 && y<0){multiplier=Math.PI; minus=1;}
        if(x<0 && y>=0){multiplier=Math.PI*2; minus=-1;}

        if(y==0)return Math.atan(x*2e9);
        return ((Math.abs(Math.atan(x/y))*minus+2*Math.PI)+multiplier)%(Math.PI*2);
    }


}
