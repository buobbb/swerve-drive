package org.firstinspires.ftc.teamcode.htech;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.htech.config.SwerveHardwareTest;
import org.firstinspires.ftc.teamcode.htech.robot.RobotSystems;
import org.firstinspires.ftc.teamcode.htech.swerve.SwerveDrivetrain;
import org.firstinspires.ftc.teamcode.htech.utils.Pose;

@Config
@Autonomous(name = "AutoBengos")
public class AutoBengos extends LinearOpMode {

    SwerveDrivetrain drivetrain;
    ElapsedTime timer;

    enum STATES{
        DRIVE,
        SHOOT,
        DA_TE_LA_O_PARTE
    }
    STATES cs = STATES.DRIVE;

    boolean finish = false;

    public static double powerX = 0.5, powerY = 0;

    public static double timeToDrive = 1000;
    public static double timeToReveneala = 1000;

    boolean firstTime = false;
    int i = 1;
    RobotSystems r;

    @Override
    public void runOpMode() throws InterruptedException {
        SwerveHardwareTest.init(hardwareMap);
        drivetrain = new SwerveDrivetrain();
        r = new RobotSystems(hardwareMap);
        timer = new ElapsedTime();

        waitForStart();

        timer.reset();

        while (opModeIsActive() && !finish){ {

            Pose pose = new Pose(powerX,powerY,0);

            switch (cs){

                case DRIVE:
                    if(timer.milliseconds() < timeToDrive){
                        drivetrain.read();
                        drivetrain.set(pose);
                        drivetrain.write();
                        drivetrain.updateModules();
                    }
                    else{
                        r.s_CS = RobotSystems.ShooterStates.CHARGING_UP;
                        r.launcher.shootAuto();
                        drivetrain.read();
                        drivetrain.set(new Pose(0,0,0));
                        drivetrain.write();
                        drivetrain.updateModules();
                        cs = STATES.SHOOT;
                        firstTime = true;
                        timer.reset();
                    }
                    break;

                case SHOOT:
                    if(timer.milliseconds() > timeToReveneala){
                        if(r.s_CS == RobotSystems.ShooterStates.WAITING_FOR_BUTTON && i <= 3){
                            r.buttonPressed = true;
                            i++;
                        }
                        else if(i > 3){
                             r.s_CS = RobotSystems.ShooterStates.IDLE;
                             r.launcher.stop();
                             cs = STATES.DA_TE_LA_O_PARTE;
                             timer.reset();
                        }
                    }
                    break;

                case DA_TE_LA_O_PARTE:
                    if(timer.milliseconds() < timeToDrive){
                        drivetrain.read();
                        drivetrain.set(new Pose(-powerX,-powerY,0));
                        drivetrain.write();
                        drivetrain.updateModules();
                    }
                    else{
                        drivetrain.read();
                        drivetrain.set(new Pose(0,0,0));
                        drivetrain.write();
                        drivetrain.updateModules();
                        finish = true;
                    }
                    break;


            }

            r.update();

        }
    }
    }
}
