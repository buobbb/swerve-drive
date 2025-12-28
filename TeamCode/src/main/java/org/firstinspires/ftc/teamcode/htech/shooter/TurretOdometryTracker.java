package org.firstinspires.ftc.teamcode.htech.shooter;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.VoltageSensor;

import org.firstinspires.ftc.teamcode.blob.localization.Odometry;
import org.firstinspires.ftc.teamcode.htech.swerve.MathUtils;

@Config
public class TurretOdometryTracker {

    public static double targetX = -118;
    public static double targetY = 20.5;
    public static double maxTurretDegrees = 90;
    public static double headingToleranceDeg = 1.0;
    public static boolean clampTurretRange = true;
    public static boolean autoUpdateOdometry = true;

    private final Odometry odometry;
    private final Turret turret;
    private final ShooterMath shooterMath = new ShooterMath();

    public TurretOdometryTracker(Odometry odometry, Turret turret) {
        this(odometry, turret, null);
    }

    public TurretOdometryTracker(Odometry odometry, Turret turret, VoltageSensor voltageSensor) {
        this.odometry = odometry;
        this.turret = turret;
        if (voltageSensor != null) {
            turret.setVoltageSensor(voltageSensor);
            Turret.voltageCompensation = true;
        }
    }

    public void setTarget(double x, double y) {
        targetX = x;
        targetY = y;
    }

    public boolean trackGoal = false;

    public void update() {
        if (autoUpdateOdometry) {
            odometry.update();
        }

        double turretAngle = getTurretAngle(targetX, targetY);
        if(trackGoal) turret.setPosition(shooterMath.angleToTicks(turretAngle));
        else turret.setPosition(0);
        turret.update();
    }

    public boolean isAligned() {
        double turretAngle = getTurretAngle(targetX, targetY);
        int targetTicks = shooterMath.angleToTicks(turretAngle);
        int toleranceTicks = Math.max(1, Math.abs(shooterMath.angleToTicks(Math.toRadians(headingToleranceDeg))));
        return Math.abs(targetTicks - turret.currentPosition) <= toleranceTicks;
    }

    public double getTurretAngle(double x, double y) {
        double dx = x - odometry.getX();
        double dy = y - odometry.getY();

        double targetHeading = Math.atan2(dy, dx) - Math.PI;
        double robotHeading = wrap(odometry.getHeading());
        double turretAngle = wrap(targetHeading - robotHeading);

        if (clampTurretRange) {
            turretAngle = MathUtils.clamp(
                    turretAngle,
                    Math.toRadians(-maxTurretDegrees),
                    Math.toRadians(maxTurretDegrees)
            );
        }

        return turretAngle;
    }

    private double wrap(double angle) {
        while (angle > Math.PI) angle -= 2 * Math.PI;
        while (angle < -Math.PI) angle += 2 * Math.PI;
        return angle;
    }
}