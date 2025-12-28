package org.firstinspires.ftc.teamcode.blob.driveTrain;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.VoltageSensor;

import org.firstinspires.ftc.teamcode.blob.localization.Odometry;
import org.firstinspires.ftc.teamcode.blob.math.PIDControllerBlob;
import org.firstinspires.ftc.teamcode.htech.config.SwerveHardwareTest;
import org.firstinspires.ftc.teamcode.htech.swerve.MathUtils;
import org.firstinspires.ftc.teamcode.htech.swerve.SlewRateLimiter;
import org.firstinspires.ftc.teamcode.htech.swerve.SwerveDrivetrain;
import org.firstinspires.ftc.teamcode.htech.utils.Pose;

@Config
public class MuieFtc{

    public static double kP = 0.05, kI = 0.0, kD = 0.001;
    public static double hP = 1.1, hI = 0.0, hD = 0.005;

    double voltage;

    PIDControllerBlob controller, hController;

    public static double maxPower = 1;
    public static double maxHeadingPower = 1;

    public static double trMaxError = 1;
    public static double hMaxError = 3;

    SwerveDrivetrain drive;
    public Odometry odo;
    VoltageSensor vs;

    SlewRateLimiter translationalSlewRateLimiter, headingSlewRateLimiter;

    public static double translationalSlewLimit = 9, headingSlewLimit = 7;

    public static double voltageConstant = 12;

    double realHeading, error, x, y, rotation;

    double targetX, targetY, targetH;

    public MuieFtc(HardwareMap hardwareMap){
        SwerveHardwareTest.init(hardwareMap);
        drive = new SwerveDrivetrain();
        odo = new Odometry(SwerveHardwareTest.vs, SwerveHardwareTest.odo);
        vs = SwerveHardwareTest.vs;

        translationalSlewRateLimiter = new SlewRateLimiter(translationalSlewLimit);
        headingSlewRateLimiter = new SlewRateLimiter(headingSlewLimit);

        controller = new PIDControllerBlob(kP, kI, kD);
        hController = new PIDControllerBlob(hP, hI, hD);

    }

    public void init(){
        drive.read();
        drive.set(new Pose(0, 0, 0));
        drive.write();
        drive.updateModules();
    }

    public void setTargetPose(Pose pose){
        targetX = pose.x;
        targetY = pose.y;
        targetH = pose.heading;
    }

    public boolean inPosition(){
        double heading = odo.getHeading();
        if(heading < 0) realHeading = Math.abs(heading);
        else realHeading = 2 * Math.PI - heading;

        error = targetH - realHeading;
        if(Math.abs(error) > Math.PI) {
            error = -Math.signum(error) * (2 * Math.PI - Math.abs(error));
        }

        return Math.abs(targetX - odo.getX()) < trMaxError &&
                Math.abs(targetY - odo.getY()) < trMaxError &&
                Math.abs(error) < Math.toRadians(hMaxError);
    }

    int i = 10;
    public void update(){

        if(i == 10){
            voltage = vs.getVoltage();
            i = 0;
        }

        odo.update();
        drive.read();

        controller.kp = kP;
        controller.ki = kI;
        controller.kd = kD;

        hController.kp = hP;
        hController.ki = hI;
        hController.kd = hD;

        if(Double.isNaN(odo.x) || Double.isNaN(odo.y) || Double.isNaN(odo.heading)){
            return;
        }

        x = controller.calculate(targetX, odo.getX());
        y = controller.calculate(targetY, odo.getY());
        rotation = hController.calculate(0 , error);

        double xRotated = x * Math.cos(odo.getHeading()) - y * Math.sin(odo.getHeading());
        double yRotated = x * Math.sin(odo.getHeading()) + y * Math.cos(odo.getHeading());

        double xPower = MathUtils.clamp(xRotated, -maxPower, maxPower);
        double yPower = MathUtils.clamp(yRotated, -maxPower, maxPower);
        double hPower = MathUtils.clamp(rotation, -maxHeadingPower, maxHeadingPower);

        if(Math.abs(xPower) < 0.01) xPower = 0;
        if(Math.abs(yPower) < 0.01) yPower = 0;

        xPower = translationalSlewRateLimiter.calculate(xPower);
        yPower = translationalSlewRateLimiter.calculate(yPower);
        hPower = translationalSlewRateLimiter.calculate(hPower);

        drive.set(new Pose(xPower * voltageConstant / voltage, -yPower * voltageConstant / voltage, hPower * voltageConstant / voltage));
        drive.write();
        drive.updateModules();


        i++;
    }



}
