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

    public static double kP = 0.058;
    public static double kI = 0.0;
    public static double kD = 0.01;

    public static double H_KP = 1;
    public static double H_KI = 0.0;
    public static double H_KD = 0;

    public static double MAX_TRANSLATION = 0.8;
    public static double MAX_ROTATION = 0.8;
    public static double LINEAR_ACCEL_LIMIT = 6.0;
    public static double ANGULAR_ACCEL_LIMIT = 6.0;
    public static double MIN_RAMP = 0.2;
    public static double NOMINAL_VOLTAGE = 12.0;

    private final SwerveDrivetrain drivetrain;
    private final Odometry odometry;
    private VoltageSensor voltageSensor;

    public double robotHeading;

    public SlewRateLimiter fwLimiter, strLimiter, headingLimiter;

    private final PIDControllerBlob translationalPID = new PIDControllerBlob(kP, kI, kD);
    private final PIDControllerBlob controllerHeading = new PIDControllerBlob(H_KP, H_KI, H_KD);
    private Pose targetPose = new Pose();

    public static double minPower= 0.1;

    public static double HEADING_OFF_DISTANCE = 16;
    public static double HEADING_ON_DISTANCE = 4;

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
        translationalPID.kp = kP;
        translationalPID.ki = kI;
        translationalPID.kd = kD;

        controllerHeading.kp = H_KP;
        controllerHeading.ki = H_KI;
        controllerHeading.kd = H_KD;

        Pose robotPose = odometry.getPos();
        Pose deltaPose = targetPose.subtract(robotPose);
        double headingError = AngleUnit.normalizeRadians(deltaPose.heading);

        double translationalError = Math.hypot(deltaPose.x, deltaPose.y);
        double headingMagnitude = Math.abs(headingError);
        double dirX = 0;
        double dirY = 0;
        if (translationalError > 0) {
            dirX = deltaPose.x / translationalError;
            dirY = deltaPose.y / translationalError;
        }

        double translationSpeed = translationalPID.calculate(0, translationalError);
        translationSpeed = MathUtils.clamp(translationSpeed, -MAX_TRANSLATION, MAX_TRANSLATION);
        translationSpeed = fwLimiter.calculate(translationSpeed);
        double xPowerField = dirX * translationSpeed;
        double yPowerField = dirY * translationSpeed;
        double headingPower = 0;
        if(translationalError < HEADING_OFF_DISTANCE){
            double headingBlend = MathUtils.clamp((HEADING_OFF_DISTANCE - translationalError)/(HEADING_OFF_DISTANCE - HEADING_ON_DISTANCE), 0, 1);

            headingBlend *= headingBlend;
            headingPower = headingBlend*controllerHeading.calculate(0, headingError);
        }

//        double translationRamp = rampScale(translationalError, TRANSLATION_SLOW_RADIUS, TRANSLATIONAL_TOLERANCE);
//        double headingRamp = rampScale(headingMagnitude, HEADING_SLOW_RADIUS, HEADING_TOLERANCE);

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