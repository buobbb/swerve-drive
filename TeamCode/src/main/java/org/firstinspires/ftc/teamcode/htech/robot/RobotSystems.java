package org.firstinspires.ftc.teamcode.htech.robot;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.VoltageSensor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.blob.localization.Odometry;
import org.firstinspires.ftc.teamcode.htech.config.SwerveHardwareTest;
import org.firstinspires.ftc.teamcode.htech.shooter.Launcher;
import org.firstinspires.ftc.teamcode.htech.shooter.Pitch;
import org.firstinspires.ftc.teamcode.htech.shooter.Turret;
import org.firstinspires.ftc.teamcode.htech.shooter.TurretOdometryTracker;
import org.firstinspires.ftc.teamcode.htech.shooter.TurretPidTx;

@Config
public class RobotSystems {

    public static double targetX = -117;
    public static double targetY = 20.5;

    public Intake intake;
    public Launcher launcher;
    public ColorSensor cs;

//    public TurretPidTx turret;
    public ElapsedTime timer;
    public Turret turret;
    public Odometry odo;
    public TurretOdometryTracker tracker;
    public VoltageSensor vs;
    public Pitch pitch;

    public double distance;

    public boolean rumbling = false;

    public static double plm = 100;
    public static double powerPlm = -0.3;

    public double voltage = 14;

    public enum ShooterStates {
        IDLE,
        CHARGING_UP,
        WAITING_FOR_BUTTON,
        CHECK_VELOCITY
    }
    public ShooterStates s_CS = ShooterStates.IDLE;

    public RobotSystems(HardwareMap hardwareMap){
        vs = SwerveHardwareTest.vs;
        pitch = new Pitch(hardwareMap);
        cs = new ColorSensor(hardwareMap);
        intake = new Intake(hardwareMap);
        launcher = new Launcher(hardwareMap);
        turret = new Turret(hardwareMap);
        turret.setVoltageSensor(vs);
        Turret.voltageCompensation = true;
        odo = new Odometry(vs, SwerveHardwareTest.odo);
        tracker = new TurretOdometryTracker(odo, turret, vs);
        timer = new ElapsedTime();
    }

    public static double timeToResetEncoder = 200;
    boolean resetTurretFirstTime = true;

    public enum resetTurretStates{

        IDLE,
        START,
        FINISH

    }
    public resetTurretStates rt_cs = resetTurretStates.IDLE;

    public void resetTurret(){
        rt_cs = resetTurretStates.START;
        resetTurretFirstTime = true;
    }

    public void updateResetTurret(){


        switch (rt_cs){

            case IDLE:
                break;

            case START:
                if(resetTurretFirstTime){
                    tracker.trackGoal = false;
                    timer.reset();
                    resetTurretFirstTime = false;
                }
                else{
                    if(timer.milliseconds() > 300){
                        odo.reset();
                        timer.reset();
                        rt_cs = resetTurretStates.FINISH;
                    }
                }
                break;

            case FINISH:
                if(timer.milliseconds() > timeToResetEncoder){
                    tracker.trackGoal = true;
                    rumbling = true;
                    rt_cs = resetTurretStates.IDLE;
                }
                break;

        }



    }

    public void update(){

        if(i == 10){
            voltage = vs.getVoltage();
            i = 0;
        }
        launcher.update();
        tracker.setTarget(targetX, targetY);
        tracker.update();
        updateShooter();
        updateResetTurret();


        distance = Math.hypot(targetX - odo.getX(), targetY - odo.getY());
    }


    int i = 0;
    int j = 1;
    boolean firstTime = true;

    public void shoot(){
        tracker.trackGoal = true;
        s_CS = ShooterStates.CHARGING_UP;
        j = 1;
    }

    void updateShooter() {
        switch (s_CS) {
            case IDLE:
                break;
            case CHARGING_UP:
                if(j <= 3){
                    if(launcher.isAtTargetSpeed1()) {
                        rumbling = true;
                        s_CS = ShooterStates.WAITING_FOR_BUTTON;
                        buttonPressed = true; //
                    }
                }
                else{
                    s_CS = ShooterStates.IDLE;
                    tracker.trackGoal = false;
                    launcher.stop();
                }
                break;
            case WAITING_FOR_BUTTON:
                if(buttonPressed) {
                    buttonPressed = false;
                    intake.mananca();
                    s_CS = ShooterStates.CHECK_VELOCITY;
                    firstTime = true;
                }
                break;
            case CHECK_VELOCITY:
                if(!launcher.isAtTargetSpeed2()) {

                    if(firstTime){
                        intake.setPower(powerPlm);
                        timer.reset();
                        firstTime = false;
                    }
                    else{
                        if(timer.milliseconds() > plm){
                            intake.stop();
                            s_CS = ShooterStates.CHARGING_UP;
                            j++;
                        }
                    }

                }
                break;
        }
    }

    public boolean buttonPressed = false;




}
