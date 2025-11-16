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
import org.firstinspires.ftc.teamcode.htech.config.mechanisms_hardware.Sensors;


@Config
public class Shooter {

    Pitch pitch;
    Turret turret;
    Launcher launcher;
    Limelight3A ll;
    LLResult result;
    ShooterMath math;
    Odometry odo;

    public static double goalPosX = 0;
    public static double goalPosY = 0;
    public static double pitchAngle = 30;
    double distance;
    double robotX, robotY, robotH;

    public Shooter(HardwareMap hardwareMap){

        math = new ShooterMath();
        pitch = new Pitch(hardwareMap);
        turret = new Turret(hardwareMap);
        launcher = new Launcher(hardwareMap);
        ll = hardwareMap.get(Limelight3A.class, Sensors.limelight);
        odo = new Odometry(SwerveHardware.vs, SwerveHardware.odo);


        ll.start();
    }


    void aimWithCamera() {
        double distance = math.updateDistance(result.getTy());
        double turretAngle = math.updateTurretAngle(result.getTx(), distance);


        double servoPos = math.angleToServoPos(Math.toRadians(pitchAngle));
        pitch.setPosition(servoPos);

        double requiredSpeed = math.calculateSpeedForDistance(distance);
        launcher.setPower(requiredSpeed);

        turret.setPosition(math.angleToTicks(turretAngle));
    }



    void aimWithOdometry() {
        double dx = goalPosX - robotX;
        double dy = goalPosY - robotY;


        distance = Math.sqrt(dx * dx + dy * dy);


        double absoluteAngleToGoal = Math.atan2(dy, dx);
        double relativeTurretAngle = math.wrapRadians(absoluteAngleToGoal - robotH);


        double servoPos = math.angleToServoPos(ShooterMath.fixedTheta);
        pitch.setPosition(servoPos);


        double requiredSpeed = math.calculateSpeedForDistance(distance);
        launcher.setPower(requiredSpeed);


        turret.setPosition(math.angleToTicks(relativeTurretAngle));
    }




    public void update(){

        result = ll.getLatestResult();

        if(result != null && result.isValid()){

            aimWithCamera();

        }
        else{

            robotX = odo.getX();
            robotY = odo.getY();
            robotH = odo.getHeading();

            aimWithOdometry();


        }

        turret.update();
        launcher.update();
        odo.update();

    }






}
