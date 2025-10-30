package org.firstinspires.ftc.teamcode.htech.shooter;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.DcMotorEx;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.htech.swerve.MathUtils;

@Config
public class ShooterMath {

    public static double hGoal = 0.985;
    public static double hTag = 0.75;
    /*TODO: */ public static double hCam = 0;
    public static double hShooter = 0.338;
    /*TODO: */ public static double llLateralMountError = 0;
    public static double g = 9.81; //acceleratie gravitationala
    public static double speed; //launcher.getVelocity() in m/s
    /*TODO: */ public static double llMountAngle = 0;  //rad, fata de orizontala


    public double distance, pitchAngle, turretAngle;

    public void updateDistance(double ty){
        distance = (hTag - hCam) / Math.tan(llMountAngle + ty);
    }

    public void updateTurretAngle(double tx){
        turretAngle = -tx + Math.atan2(llLateralMountError, distance);
    }

    public void updatePitchAngle(){
        pitchAngle = Math.atan(speed * speed - Math.sqrt(speed * speed * speed * speed - g * (g * distance * distance + 2 * (hGoal - hShooter) * speed * speed)) / (g * distance));
    }



    //convertire pos servo --> unghi    pitch
    /*TODO: */ public static double s_min = 0; //min pos servo
    /*TODO: */ public static double s_max = 1; //max pos servo
    /*TODO: */ public static double theta_min = Math.toRadians(0); //min pos shooter
    /*TODO: */ public static double theta_max = Math.toRadians(60); //max pos shooter


    public double angleToServoPos(double radians){

        radians = MathUtils.clamp(radians, theta_min, theta_max);
        double s_target = s_min + (radians - theta_min) * (s_max - s_min) / (theta_max - theta_min);
        return MathUtils.clamp(s_target, 0, 1);

    }


    //convertire pos motor --> unghi   turela
    /* TODO: */ public static double ticks_per_turret_rev = 0; //motor.getMotorType().getTicksPerRev() * raport transmisie;
    /* TODO: */ public static double ticks_zero = 0; //pozitia motorului cand turela e orientata inspre fata robotului
    /* TODO: */ public static int dir = 1; //directia

    public int angleToTicks(double radians){

        return (int) (ticks_zero + dir * (radians / (2 * Math.PI)) * ticks_per_turret_rev);

    }

    public double wrapRadians(double angle) {
        angle = (angle + Math.PI) % (2 * Math.PI);
        if (angle < 0)
            angle += 2 * Math.PI;
        return angle - Math.PI;
    }







}
