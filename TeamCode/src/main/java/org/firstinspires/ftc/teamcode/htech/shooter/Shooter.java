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
    ShooterMathCpp math;
    Odometry odo;

    public static double goalPosX = 0;
    public static double goalPosY = 0;
    double distance, pitchAngle, turretAngle;
    double robotX, robotY, robotH;

    public Shooter(HardwareMap hardwareMap){

        math = new ShooterMathCpp();
        pitch = new Pitch(hardwareMap);
        turret = new Turret(hardwareMap);
        launcher = new Launcher(hardwareMap);
        ll = hardwareMap.get(Limelight3A.class, Sensors.limelight);
        odo = new Odometry(SwerveHardware.vs, SwerveHardware.odo);


        ll.start();
    }


    void aimWithCamera() {
        distance = math.updateDistance(result.getTy());
        turretAngle = math.updateTurretAngle(result.getTx(), distance);
        pitchAngle = math.updatePitchAngle(distance);

        pitch.setPosition(math.angleToServoPos(pitchAngle));
        turret.setPosition(math.angleToTicks(turretAngle));
    }


    void aimWithOdometry() {
        double dx = goalPosX - robotX;
        double dy = goalPosY - robotY;

        distance = Math.sqrt(dx * dx + dy * dy);
        double absoluteAngleToGoal = Math.atan2(dy, dx);
        double relativeTurretAngle = math.wrapRadians(absoluteAngleToGoal - robotH);

        pitchAngle = math.updatePitchAngle(distance);

        pitch.setPosition(math.angleToServoPos(pitchAngle));
        turret.setPosition(math.angleToTicks(relativeTurretAngle));
    }



    public void update(){

        result = ll.getLatestResult();
        math.setSpeed(launcher.mathVelocity);

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
