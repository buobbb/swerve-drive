package org.firstinspires.ftc.teamcode.htech.robot;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.VoltageSensor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.htech.config.SwerveHardwareTest;
import org.firstinspires.ftc.teamcode.htech.shooter.Launcher;
import org.firstinspires.ftc.teamcode.htech.shooter.Pitch;
import org.firstinspires.ftc.teamcode.htech.shooter.TurretPidTx;

@Config
public class RobotSystems {

    public Intake intake;
    public Launcher launcher;
    public ColorSensor cs;

    public TurretPidTx turret;
    public ElapsedTime timer;
    public VoltageSensor vs;
    public Pitch pitch;

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
        turret = new TurretPidTx(hardwareMap);
        timer = new ElapsedTime();
    }

    public void update(){

        if(i == 10){
            voltage = vs.getVoltage();
            i = 0;
        }
        launcher.update();
        turret.update(voltage);
        updateShooter();
    }


    int i = 0;
    boolean firstTime = true;
    void updateShooter() {
        switch (s_CS) {
            case IDLE:
                break;
            case CHARGING_UP:
                turret.readLl = true;
                if(launcher.isAtTargetSpeed1()) {
                    rumbling = true;
                    s_CS = ShooterStates.WAITING_FOR_BUTTON;
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
                        }
                    }

                }
                break;
        }
    }

    public boolean buttonPressed = false;




}
