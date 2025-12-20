package org.firstinspires.ftc.teamcode.htech.shooter;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.VoltageSensor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.blob.localization.GoBildaPinpointDriver;
import org.firstinspires.ftc.teamcode.blob.localization.Odometry;
import org.firstinspires.ftc.teamcode.htech.config.SwerveHardware;
import org.firstinspires.ftc.teamcode.htech.config.SwerveHardwareTest;
import org.firstinspires.ftc.teamcode.htech.config.mechanisms_hardware.Sensors;
import org.firstinspires.ftc.teamcode.htech.swerve.MathUtils;


@Config
public class Shooter {

    public Pitch pitch;
    public Turret turret;
    public Launcher launcher;
    public ShooterMath math;
    public Odometry odo;

    public static double maxTurret = 90;

    public static double goalPosX = -118;
    public static double goalPosY = 20.5;
    public static double pitchAngle = 30;

    public static double hoodClosePos = 0.5;
    public static double hoodFarPos = 1;

    public static double thresholdHood = 58.5;

    public double distance;
    public double robotX, robotY, robotH;
    public double turretAngle;

    public Shooter(HardwareMap hardwareMap){

        math = new ShooterMath();
        pitch = new Pitch(hardwareMap);
        turret = new Turret(hardwareMap);
        launcher = new Launcher(hardwareMap);
        odo = new Odometry(SwerveHardwareTest.vs, SwerveHardwareTest.odo);


    }

    public static double normalizeAngle(double angle){
        while(angle > Math.PI) angle -= 2 * Math.PI;
        while(angle < -Math.PI) angle += 2 * Math.PI;
        return angle;
    }

    int i = 1;

    public void update(){

        odo.update();

        robotX = odo.getX();
        robotY = odo.getY();
        robotH = normalizeAngle(odo.getHeading());

        double x = goalPosX - robotX;
        double y = goalPosY - robotY;

        distance = Math.hypot(x, y);

        turretAngle = Math.atan(y / x) - robotH;
        turretAngle = normalizeAngle(turretAngle);
        turretAngle = MathUtils.clamp(turretAngle, Math.toRadians(-maxTurret), Math.toRadians(maxTurret));
        turret.setPosition(math.angleToTicks(turretAngle));





        turret.update();
        launcher.update();


    }






}
