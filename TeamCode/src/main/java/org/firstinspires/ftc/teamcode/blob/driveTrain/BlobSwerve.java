package org.firstinspires.ftc.teamcode.blob.driveTrain;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.VoltageSensor;

import org.firstinspires.ftc.teamcode.blob.constants.BlobConstants;
import org.firstinspires.ftc.teamcode.blob.localization.Odometry;
import org.firstinspires.ftc.teamcode.blob.math.PIDControllerBlob;
import org.firstinspires.ftc.teamcode.htech.config.Hardware;
import org.firstinspires.ftc.teamcode.htech.swerve.SwerveDrivetrain;
import com.pedropathing.geometry.Pose;

@Config
public class BlobSwerve {

    public SwerveDrivetrain drive;
    public Odometry odo;

    public double targetX, targetY, targetHeading;
    public double xCmd, yCmd, wCmd;
    public double realHeading, headingError;

    public double kP = BlobConstants.kP, kI = BlobConstants.kI, kD = BlobConstants.kD;
    public double hP = BlobConstants.hP, hI = BlobConstants.hI, hD = BlobConstants.hD;

    public static double MAX_CMD = 1.0;
    public static double MAX_W   = 1.0;

    public static double DEAD_XY = 0.8;
    public static double DEAD_H  = 0.1;
    public static double LATERAL_MULT = 1.00;

    private final PIDControllerBlob pidX = new PIDControllerBlob(kP, kI, kD);
    private final PIDControllerBlob pidY = new PIDControllerBlob(kP, kI, kD);
    private final PIDControllerBlob pidH = new PIDControllerBlob(hP, hI, hD);



    private boolean useHeadingGate = false;
    private double headingGateProgress = 0.4;
    private double prevX, prevY, prevH;
    private double totalDist, travelDist, progress;

    public BlobSwerve(HardwareMap hw, SwerveDrivetrain drivetrain) {
        this.drive = drivetrain;
        this.odo   = new Odometry(hw);
        setTargetPosition(0,0,0);
        Hardware.auto = true;
    }

    public void resetPose(double x, double y, double headingRad) {
        odo.reset();
        targetX = x;
        targetY = y;
        targetHeading = headingRad;
    }

    public void turnToDegrees(double deg) {
        targetHeading = Math.toRadians(deg);
    }

    public void turnToRadians(double rad) {
        targetHeading = rad;
    }

    public void setTargetPosition(double x, double y, double h) {
        targetX = x;
        targetY = y;
        targetHeading = h;
        useHeadingGate = false;
    }

    public void setTargetPosition(double x, double y) {
        targetX = x;
        targetY = y;
        useHeadingGate = false;
    }

    public void setTargetPosition(Pose pose) {
        targetX = pose.getX();
        targetY = pose.getY();
        targetHeading = pose.getHeading();
        useHeadingGate = false;
    }

    public void setTargetPosition(Pose pose, double gateProgress, double prevHeading) {
        targetX = pose.getX();
        targetY = pose.getY();
        prevH   = prevHeading;
        targetHeading = pose.getHeading();
        useHeadingGate = true;
        headingGateProgress = gateProgress;
        prevX = odo.getX();
        prevY = odo.getY();
    }

    public boolean inPosition() { return inPosition(DEAD_XY, DEAD_XY, DEAD_H); }

    public boolean inPosition(double tolX, double tolY, double tolH) {
        double h = odo.getHeading();
        realHeading = (h < 0) ? Math.abs(h) : 2*Math.PI - h;
        double th = targetHeading;
        double e = th - realHeading;
        if (Math.abs(e) > Math.PI) e = -Math.signum(e) * (2*Math.PI - Math.abs(e));
        headingError = e;

        return Math.abs(targetX - odo.getX()) < tolX &&
                Math.abs(targetY - odo.getY()) < tolY &&
                Math.abs(headingError) < tolH;
    }

    public void update() {
        odo.update();

        if (Double.isNaN(odo.x) || Double.isNaN(odo.y) || Double.isNaN(odo.heading)) return;

        pidX.kp = kP; pidX.ki = kI; pidX.kd = kD;
        pidY.kp = kP; pidY.ki = kI; pidY.kd = kD;
        pidH.kp = hP; pidH.ki = hI; pidH.kd = hD;

        double h = odo.getHeading();
        realHeading = (h < 0) ? Math.abs(h) : 2*Math.PI - h;
        double eH = targetHeading - realHeading;
        if (Math.abs(eH) > Math.PI) eH = -Math.signum(eH) * (2*Math.PI - Math.abs(eH));
        headingError = eH;

        double vx = pidX.calculate(targetX, odo.predictedX);
        double vy = pidY.calculate(targetY, odo.predictedY);
        double vw = pidH.calculate(eH, 0);

        vx *= LATERAL_MULT;

        double rx =  vy * Math.cos(-h) - vx * Math.sin(-h);
        double ry =  vy * Math.sin(-h) + vx * Math.cos(-h);

        rx = clamp(rx, -MAX_CMD, MAX_CMD);
        ry = clamp(ry, -MAX_CMD, MAX_CMD);
        vw = clamp(vw, -MAX_W, MAX_W);

        if (useHeadingGate) {
            totalDist  = Math.hypot(targetX - prevX, targetY - prevY);
            travelDist = Math.hypot(odo.getX() - prevX,  odo.getY() - prevY);
            progress   = (totalDist <= 1e-6) ? 1.0 : clamp(travelDist / totalDist, 0, 1);
            double desiredH = (progress < headingGateProgress) ? prevH : targetHeading;
            double eGate = desiredH - realHeading;
            if (Math.abs(eGate) > Math.PI) eGate = -Math.signum(eGate) * (2*Math.PI - Math.abs(eGate));
            vw = clamp(pidH.calculate(eGate, 0), -MAX_W, MAX_W);
        }

        xCmd = rx; yCmd = ry; wCmd = vw;

        drive.set(new org.firstinspires.ftc.teamcode.htech.utils.Pose(xCmd, yCmd, wCmd));
        drive.write();
        drive.updateModules();
    }

    private double clamp(double v, double lo, double hi) {
        return Math.max(lo, Math.min(hi, v));
    }

}
