package org.firstinspires.ftc.teamcode.htech.shooter;

import com.acmerobotics.dashboard.config.Config;

@Config
public class ShooterMathCpp {

    static {
        System.loadLibrary("shootermath");
    }

    // --- Native function declarations ---
    public native double updateDistance(double ty);
    public native double updateTurretAngle(double tx, double distance);
    public native double updatePitchAngle(double distance);
    public native double angleToServoPos(double radians);
    public native int angleToTicks(double radians);
    public native double wrapRadians(double angle);

    // --- Setters for constants ---
    public native void setSpeed(double s);
    public native void setLLMountAngle(double angle);
    public native void setHGoal(double val);
    public native void setHShooter(double val);
    public native void setHTag(double val);
    public native void setHCam(double val);
    public native void setLLLateralMountError(double val);
    public native void setServoParams(double sMin, double sMax, double tMin, double tMax);
    public native void setTurretParams(double ticksPerRev, double ticksZero, int direction);
}
