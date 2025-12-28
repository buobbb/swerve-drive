package org.firstinspires.ftc.teamcode.blob.driveTrain;

import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.controller.PIDFController;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.blob.localization.Odometry;
import org.firstinspires.ftc.teamcode.blob.math.PIDControllerBlob;
import org.firstinspires.ftc.teamcode.htech.swerve.MathUtils;
import org.firstinspires.ftc.teamcode.htech.swerve.SlewRateLimiter;
import org.firstinspires.ftc.teamcode.htech.swerve.SwerveDrivetrain;
import org.firstinspires.ftc.teamcode.htech.utils.Pose;

@Config
public class BlobDecode {

    public double PowerX, PowerY, PowerH;
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

    public static PIDControllerBlob xController = new PIDControllerBlob(xP, 0, xD);
    public static PIDControllerBlob yController = new PIDControllerBlob(yP, 0.0, yD);
    public static PIDControllerBlob hController = new PIDControllerBlob(hP, 0.0, hD);
    public static double max_power = 1;
    public static double max_heading = 0.5;

    SwerveDrivetrain drivetrain;
    public Odometry localizer;
    Pose targetPose;
    ElapsedTime deadTimer;
    private ElapsedTime delayTimer;

    SlewRateLimiter tr, h;

    public static double slewTr = 7;
    public static double slewH = 4;

    private double v = 12;

    public Pose robotPose;

    public BlobDecode(SwerveDrivetrain drivetrain, Odometry localizer, Pose targetPose) {
        this.drivetrain = drivetrain;
        this.localizer = localizer;
        this.targetPose = targetPose;

        tr = new SlewRateLimiter(slewTr);
        h = new SlewRateLimiter(slewH);

    }

    public void init(){
        localizer.setPoseEstimate(targetPose.getX(), targetPose.getY(), targetPose.heading);
        robotPose = localizer.getPos();
        drivetrain.read();
        drivetrain.set(targetPose);
        drivetrain.write();
        drivetrain.updateModules();
    }

    public void update() {
        v = localizer.getVoltage();

        xController.kp = xP;
        xController.ki = 0;
        xController.kd = xD;

        yController.kp = yP;
        yController.ki = 0;
        yController.kd = yD;

        hController.kp = hP;
        hController.ki = 0;
        hController.kd = hD;

        localizer.update();
        robotPose = localizer.getPos();
        drivetrain.read();
        Pose deltaPose = relDistanceToTarget(robotPose, targetPose);
        double translationalError = Math.hypot(deltaPose.getX(), deltaPose.getY());
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

        PowerX = tr.calculate(x_power);
        PowerY = tr.calculate(y_power);
        PowerH = h.calculate(heading_power);

        Pose powers1 = new Pose(-PowerY, PowerX, -PowerH);
        drivetrain.set(powers1);
        drivetrain.write();
        drivetrain.updateModules();
    }

    public boolean inPosition() {
        Pose error = targetPose.subtract(localizer.getPos());

        return ((Math.hypot(error.x, error.y) < ALLOWED_TRANSLATIONAL_ERROR) && (Math.abs(error.heading) < ALLOWED_HEADING_ERROR));

    }

    public void setTargetPose(Pose targetPose){
        this.targetPose = targetPose;
    }

    private double getVoltageScale() {
        if (v == 0) {
            return 1.0;
        }
        return 12 / v;
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