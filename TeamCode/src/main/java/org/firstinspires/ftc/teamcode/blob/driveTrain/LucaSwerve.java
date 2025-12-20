package org.firstinspires.ftc.teamcode.blob.driveTrain;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.PIDFController;
import org.firstinspires.ftc.teamcode.blob.localization.Odometry;
import org.firstinspires.ftc.teamcode.htech.swerve.MathUtils;
import org.firstinspires.ftc.teamcode.htech.swerve.SwerveDrivetrain;
import org.firstinspires.ftc.teamcode.htech.utils.Pose;

@Config
public class LucaSwerve{
    public static double ALLOWED_TRANSLATIONAL_ERROR = 0.25;
    public static double ALLOWED_HEADING_ERROR = Math.toRadians(1);

    public static double xP = 0.04;
    public static double xD = 0.05;
    public static double xF = 0;

    public static double yP = 0.04;
    public static double yD = 0.05;
    public static double yF = 0;

    public static double hP = 0.6;
    public static double hD = 0.3;
    public static double hF = 0;

    public static PIDFController xController = new PIDFController(xP, 0.0, xD, xF);
    public static PIDFController yController = new PIDFController(yP, 0.0, yD, yF);
    public static PIDFController hController = new PIDFController(hP, 0.0, hD, hF);
    public static double max_power = 1;
    public static double max_heading = 0.5;

    SwerveDrivetrain drivetrain;
    public Odometry localizer;
    Pose targetPose;
    ElapsedTime deadTimer;
    private ElapsedTime delayTimer;

    private double v = 14;

    public LucaSwerve(SwerveDrivetrain drivetrain, Odometry localizer, Pose targetPose) {
        this.drivetrain = drivetrain;
        this.localizer = localizer;
        this.targetPose = targetPose;
    }

    public void init(){
        localizer.setPoseEstimate(targetPose.getX(), targetPose.getY(), targetPose.heading);
        drivetrain.read();
        drivetrain.set(targetPose);
        drivetrain.write();
        drivetrain.updateModules();
    }

    public void update() {
        v = localizer.getVoltage();
        localizer.update();
        drivetrain.read();
        Pose powers = goToPosition(localizer.getPos(), targetPose);
        drivetrain.set(powers);
        drivetrain.write();
        drivetrain.updateModules();

    }

    public boolean inPosition() {
        Pose error = targetPose.subtract(localizer.getPos());

        return ((Math.hypot(error.x, error.y) < ALLOWED_TRANSLATIONAL_ERROR) && (Math.abs(error.heading) < ALLOWED_HEADING_ERROR));

    }

    public void setTargetPose(Pose targetPose1){
        this.targetPose = targetPose1;
    }

    private static Pose relDistanceToTarget(Pose robot, Pose target) {
        return target.subtract(robot);
    }

    public Pose goToPosition(Pose robotPose, Pose targetPose) {
        Pose deltaPose = relDistanceToTarget(robotPose, targetPose);
        Pose powers = new Pose(
                xController.calculate(0, deltaPose.x),
                yController.calculate(0, deltaPose.y),
                hController.calculate(0, deltaPose.heading)
        );
        double x_rotated = powers.x * Math.cos(robotPose.heading) - powers.y * Math.sin(robotPose.heading);
        double y_rotated = powers.x * Math.sin(robotPose.heading) + powers.y * Math.cos(robotPose.heading);
        double x_power = -x_rotated < -max_power ? -max_power :
                Math.min(-x_rotated, max_power);
        double y_power = -y_rotated < -max_power ? -max_power :
                Math.min(-y_rotated, max_power);
        double heading_power = MathUtils.clamp(powers.heading, -max_heading, max_heading);

        if(Math.abs(x_power) < 0.01) x_power = 0;
        if(Math.abs(y_power) < 0.01) y_power = 0;

        return new Pose(-y_power / v * 12, x_power / v * 12, -heading_power / v * 12);
    }
}