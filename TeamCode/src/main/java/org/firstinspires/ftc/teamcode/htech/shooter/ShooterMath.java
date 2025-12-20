package org.firstinspires.ftc.teamcode.htech.shooter;

import com.acmerobotics.dashboard.config.Config;

@Config
public class ShooterMath {

    public static double hGoal = 100;
    public static double hTag = 74;
    public static double hCam = 0;
    public static double hShooter = 0.338;
    public static double llLateralMountError = 0;
    public static double g = 9.81;
    public static double llMountAngle = 75;


    public static double s_min = 0;
    public static double s_max = 1;
    public static double theta_min = Math.toRadians(0);
    public static double theta_max = Math.toRadians(60);
    public static double fixedTheta = Math.toRadians(30);
    public static double ticks_per_turret_rev = 3440;
    public static double ticks_zero = 0;
    public static int dir = 1;



    public double updateDistance(double ty) {
        return (hTag - hCam) / Math.tan(llMountAngle + ty);
    }

    public double updateTurretAngle(double tx, double distance) {
        return -tx + Math.atan2(llLateralMountError, distance);
    }

    public double calculateSpeedForDistance(double distance) {
        double num = g * distance * distance;
        double den = 2 * Math.pow(Math.cos(fixedTheta), 2) *
                (distance * Math.tan(fixedTheta) - (hGoal - hShooter));

        if (den <= 0) return Double.NaN;
        return Math.sqrt(num / den);
    }

    public double angleToServoPos(double radians) {
        radians = clamp(radians, theta_min, theta_max);
        double s_target = s_min + (radians - theta_min) * (s_max - s_min) / (theta_max - theta_min);
        return clamp(s_target, 0, 1);
    }

    public int angleToTicks(double radians) {
        return (int) ((ticks_zero + dir * (radians / (2 * Math.PI)) * ticks_per_turret_rev)) / 10;
    }

    public double wrapRadians(double angle) {
        angle = (angle + Math.PI) % (2 * Math.PI);
        if (angle < 0)
            angle += 2 * Math.PI;
        return angle - Math.PI;
    }

    private double clamp(double x, double min, double max) {
        return Math.max(min, Math.min(max, x));
    }
}
