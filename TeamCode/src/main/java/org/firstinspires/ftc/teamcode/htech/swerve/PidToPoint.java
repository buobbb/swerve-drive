package org.firstinspires.ftc.teamcode.htech.swerve;

import static java.lang.Math.cos;
import static java.lang.Math.sin;

import com.acmerobotics.dashboard.config.Config;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.PIDFController;
import org.firstinspires.ftc.teamcode.blob.localization.Odometry;
import org.firstinspires.ftc.teamcode.htech.swerve.MathUtils;
import org.firstinspires.ftc.teamcode.htech.utils.Pose;

@Config
public class PidToPoint {
    public static double TRANSLATIONAL_TOLERANCE = 0.5;
    public static double HEADING_TOLERANCE = Math.toRadians(2.0);

    public static double X_KP = 0.05;
    public static double X_KI = 0.0;
    public static double X_KD = 0.0025;
    public static double X_KF = 0.0;

    public static double Y_KP = 0.05;
    public static double Y_KI = 0.0;
    public static double Y_KD = 0.0025;
    public static double Y_KF = 0.0;

    public static double H_KP = 1.2;
    public static double H_KI = 0.0;
    public static double H_KD = 0.02;
    public static double H_KF = 0.0;

    public static double MAX_TRANSLATION = 1.0;
    public static double MAX_ROTATION = 0.6;
    public static double MINIMUM_OUTPUT = 0.02;

    private final SwerveDrivetrain drivetrain;
    private final Odometry odometry;

    private final PIDFController xController = new PIDFController(X_KP, X_KI, X_KD, X_KF);
    private final PIDFController yController = new PIDFController(Y_KP, Y_KI, Y_KD, Y_KF);
    private final PIDFController hController = new PIDFController(H_KP, H_KI, H_KD, H_KF);

    private Pose targetPose = new Pose();

    public PidToPoint(SwerveDrivetrain drivetrain, Odometry odometry) {
        this.drivetrain = drivetrain;
        this.odometry = odometry;
        xController.setTolerance(TRANSLATIONAL_TOLERANCE);
        yController.setTolerance(TRANSLATIONAL_TOLERANCE);
        hController.setTolerance(HEADING_TOLERANCE);
    }

    public void setTargetPose(Pose targetPose) {
        this.targetPose = targetPose;
        xController.setSetPoint(targetPose.x);
        yController.setSetPoint(targetPose.y);
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

        xController.setPIDF(X_KP, X_KI, X_KD, X_KF);
        yController.setPIDF(Y_KP, Y_KI, Y_KD, Y_KF);
        hController.setPIDF(H_KP, H_KI, H_KD, H_KF);

        Pose robotPose = odometry.getPos();
        double headingError = AngleUnit.normalizeRadians(targetPose.heading - robotPose.heading);

        double xPowerField = xController.calculate(robotPose.x, targetPose.x);
        double yPowerField = yController.calculate(robotPose.y, targetPose.y);
        hController.setSetPoint(0.0);
        double headingPower = hController.calculate(headingError);

        double cosH = cos(robotPose.heading);
        double sinH = sin(robotPose.heading);
        double robotX = xPowerField * cosH - yPowerField * sinH;
        double robotY = xPowerField * sinH + yPowerField * cosH;

        robotX = clampOutput(robotX, MAX_TRANSLATION);
        robotY = clampOutput(robotY, MAX_TRANSLATION);
        headingPower = clampOutput(headingPower, MAX_ROTATION);

        drivetrain.set(new Pose(robotX, robotY, headingPower));
        drivetrain.write();
        drivetrain.updateModules();
    }

    public boolean inPosition() {
        Pose error = targetPose.subtract(odometry.getPos());
        double translationalError = Math.hypot(error.x, error.y);
        double headingError = Math.abs(AngleUnit.normalizeRadians(error.heading));
        return translationalError < TRANSLATIONAL_TOLERANCE && headingError < HEADING_TOLERANCE;
    }

    private double clampOutput(double value, double maxMagnitude) {
        double clamped = MathUtils.clamp(value, -maxMagnitude, maxMagnitude);
        if (Math.abs(clamped) < MINIMUM_OUTPUT) {
            return 0.0;
        }
        return clamped;
    }
}