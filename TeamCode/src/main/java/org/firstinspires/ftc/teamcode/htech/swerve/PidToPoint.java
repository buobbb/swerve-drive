package org.firstinspires.ftc.teamcode.htech.swerve;

import static java.lang.Math.cos;
import static java.lang.Math.sin;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.VoltageSensor;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.blob.localization.Odometry;
import org.firstinspires.ftc.teamcode.blob.math.PIDControllerBlob;
import org.firstinspires.ftc.teamcode.htech.utils.Pose;

@Config
public class PidToPoint {
    public static double TRANSLATIONAL_TOLERANCE = 1.1;
    public static double HEADING_TOLERANCE = Math.toRadians(3);

    public static double X_KP = 0.05;
    public static double X_KI = 0.0;
    public static double X_KD = 0;
    public static double X_KF = 0.0;

    public static double Y_KP = 0.05;
    public static double Y_KI = 0.0;
    public static double Y_KD = 0;
    public static double Y_KF = 0.0;

    public static double H_KP = 1;
    public static double H_KI = 0.0;
    public static double H_KD = 0;
    public static double H_KF = 0.0;

    public static double MAX_TRANSLATION = 0.8;
    public static double MAX_ROTATION = 0.8;
    public static double MIN_TRANSLATION_OUTPUT = 0.02;
    public static double MIN_HEADING_OUTPUT = 0.01;
    public static double TRANSLATION_SLOW_RADIUS = 6.0;
    public static double LINEAR_ACCEL_LIMIT = 6.0;
    public static double ANGULAR_ACCEL_LIMIT = 6.0;
    public static double HEADING_SLOW_RADIUS = Math.toRadians(45.0);
    public static double MIN_RAMP = 0.2;
    public static double NOMINAL_VOLTAGE = 12.0;

    private final SwerveDrivetrain drivetrain;
    private final Odometry odometry;
    private VoltageSensor voltageSensor;

    public double robotHeading;

    public SlewRateLimiter fwLimiter, strLimiter, headingLimiter;

    private final PIDControllerBlob controllerX = new PIDControllerBlob(X_KP, X_KI, X_KD);
    private final PIDControllerBlob controllerY = new PIDControllerBlob(Y_KP, Y_KI, Y_KD);
    private final PIDControllerBlob controllerHeading = new PIDControllerBlob(H_KP, H_KI, H_KD);
    public static double X_KP_SEC = 0, X_KD_SEC = 0, Y_KP_SEC = 0, Y_KD_SEC = 0;
    public static double X_SECONDARY_ERROR = 10, Y_SECONDARY_ERROR = 10;
    private Pose targetPose = new Pose();

    public static double minPower= 0.1;

    public PidToPoint(SwerveDrivetrain drivetrain, Odometry odometry) {
        this.drivetrain = drivetrain;
        this.odometry = odometry;
        fwLimiter = new SlewRateLimiter(LINEAR_ACCEL_LIMIT);
        strLimiter = new SlewRateLimiter(LINEAR_ACCEL_LIMIT);
        headingLimiter = new SlewRateLimiter(ANGULAR_ACCEL_LIMIT);
    }

    public void setVoltageSensor(VoltageSensor voltageSensor) {
        this.voltageSensor = voltageSensor;
    }

    public void setTargetPose(Pose targetPose) {
        this.targetPose = targetPose;
    }

    public void init(Pose startPose) {
        odometry.setPoseEstimate(startPose.x, startPose.y, startPose.heading);
        drivetrain.read();
        drivetrain.set(startPose);
        drivetrain.write();
        drivetrain.updateModules();
        setTargetPose(targetPose);
    }

    public void update() {
        odometry.update();
        drivetrain.read();
        controllerX.kp = X_KP;
        controllerY.kp = Y_KP;

        controllerX.ki = X_KI;
        controllerY.ki = Y_KI;

        controllerX.kd = X_KD;
        controllerY.kd = Y_KD;

        controllerHeading.kp = H_KP;
        controllerHeading.ki = H_KI;
        controllerHeading.kd = H_KD;

        Pose robotPose = odometry.getPos();
        Pose deltaPose = targetPose.subtract(robotPose);
        double headingError = AngleUnit.normalizeRadians(deltaPose.heading);

        double xPowerField = controllerX.calculate(0, deltaPose.x);
        double yPowerField = controllerY.calculate(0, deltaPose.y);
        double headingPower = controllerHeading.calculate(0, headingError);

        double translationalError = Math.hypot(deltaPose.x, deltaPose.y);
        double headingMagnitude = Math.abs(headingError);


        double translationRamp = rampScale(translationalError, TRANSLATION_SLOW_RADIUS, TRANSLATIONAL_TOLERANCE);
        double headingRamp = rampScale(headingMagnitude, HEADING_SLOW_RADIUS, HEADING_TOLERANCE);

        double cosH = cos(robotPose.heading);
        double sinH = sin(robotPose.heading);
        double x_rotated = xPowerField * cosH - yPowerField * sinH;
        double y_rotated = xPowerField * sinH + yPowerField * cosH;

//        double robotLeft = clampOutput(yRotated, MAX_TRANSLATION * translationRamp, MIN_TRANSLATION_OUTPUT);
//        double robotForward = clampOutput(-xRotated, MAX_TRANSLATION * translationRamp, MIN_TRANSLATION_OUTPUT);
//        double robotHeading = clampOutput(-headingPower, MAX_ROTATION * headingRamp, MIN_HEADING_OUTPUT);
        double robotForward = -x_rotated < -MAX_TRANSLATION ? -MAX_TRANSLATION :
                Math.min(-x_rotated, MAX_TRANSLATION);
        double robotLeft = -y_rotated < -MAX_TRANSLATION ? -MAX_TRANSLATION :
                Math.min(-y_rotated, MAX_TRANSLATION);
        robotHeading = MathUtils.clamp(headingPower, -MAX_ROTATION, MAX_ROTATION);
        double voltageScale = getVoltageScale();
        robotForward = fwLimiter.calculate(robotForward);
        robotLeft = strLimiter.calculate(robotLeft);
        robotHeading = headingLimiter.calculate(robotHeading);

        if(robotForward < minPower && robotForward > 0) robotForward = 0;
        if(robotLeft < minPower && robotLeft > 0) robotLeft = 0;
        if(robotHeading < minPower && robotHeading > 0) robotHeading = 0;

        if(inPosition()){
            drivetrain.set(new Pose(0, 0, 0));
        }
        else{
            drivetrain.set(new Pose(robotLeft * voltageScale, -robotForward * voltageScale, -robotHeading * voltageScale));
        }

        drivetrain.write();
        drivetrain.updateModules();
    }

    public boolean inPosition() {
        Pose error = targetPose.subtract(odometry.getPos());
        double translationalError = Math.hypot(error.x, error.y);
        double headingError = Math.abs(AngleUnit.normalizeRadians(error.heading));
        return translationalError < TRANSLATIONAL_TOLERANCE && headingError < HEADING_TOLERANCE;
    }

    private double clampOutput(double value, double maxMagnitude, double minMagnitude) {
        double clamped = MathUtils.clamp(value, -maxMagnitude, maxMagnitude);
        if (Math.abs(clamped) < minMagnitude) {
            return 0.0;
        }
        return clamped;
    }

    private double getVoltageScale() {
        if (voltageSensor == null || voltageSensor.getVoltage() == 0) {
            return 1.0;
        }
        return NOMINAL_VOLTAGE / voltageSensor.getVoltage();
    }

    private double rampScale(double errorMagnitude, double slowRadius, double tolerance) {
        if (slowRadius <= 0) {
            return 1.0;
        }
        double scaled = MathUtils.clamp(errorMagnitude / slowRadius, 0.0, 1.0);
        if (scaled < MIN_RAMP && errorMagnitude > tolerance) {
            return MIN_RAMP;
        }
        return scaled;
    }
}