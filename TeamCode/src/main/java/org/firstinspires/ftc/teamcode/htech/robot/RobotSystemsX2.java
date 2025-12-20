package org.firstinspires.ftc.teamcode.htech.robot;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.htech.shooter.Launcher;
import org.firstinspires.ftc.teamcode.htech.shooter.Pitch;
import org.firstinspires.ftc.teamcode.htech.shooter.Shooter;
import org.firstinspires.ftc.teamcode.htech.shooter.TurretPidTx;

@Config

public class RobotSystemsX2 {

    public Intake intake;
    public Shooter shooter;
    public ColorSensor cs;

    public ElapsedTime timer;

    public boolean rumbling = false;
    public enum ShooterStates {
        IDLE,
        CHARGING_UP,
        WAITING_FOR_BUTTON,
        CHECK_VELOCITY
    }
    public RobotSystems.ShooterStates s_CS = RobotSystems.ShooterStates.IDLE;

    public RobotSystemsX2(HardwareMap hardwareMap){
        cs = new ColorSensor(hardwareMap);
        intake = new Intake(hardwareMap);
        shooter = new Shooter(hardwareMap);
        timer = new ElapsedTime();
    }

    public void update(){
        shooter.update();
        updateShooter();
    }


    void updateShooter() {
        switch (s_CS) {
            case IDLE:
                break;
            case CHARGING_UP:
                if(shooter.launcher.isAtTargetSpeed1()) {
                    rumbling = true;
                    s_CS = RobotSystems.ShooterStates.WAITING_FOR_BUTTON;
                }
                break;
            case WAITING_FOR_BUTTON:
                if(buttonPressed) {
                    buttonPressed = false;
                    intake.mananca();
                    s_CS = RobotSystems.ShooterStates.CHECK_VELOCITY;
                }
                break;
            case CHECK_VELOCITY:
                if(!shooter.launcher.isAtTargetSpeed2()) {
                    intake.stop();
                    s_CS = RobotSystems.ShooterStates.CHARGING_UP;
                }
                break;
        }
    }

    public boolean buttonPressed = false;

}
