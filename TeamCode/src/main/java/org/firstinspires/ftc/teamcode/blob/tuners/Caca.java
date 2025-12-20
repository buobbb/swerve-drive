package org.firstinspires.ftc.teamcode.blob.tuners;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.htech.config.SwerveHardwareTest;
import org.firstinspires.ftc.teamcode.htech.swerve.SwerveDrivetrain;
import org.firstinspires.ftc.teamcode.htech.utils.Pose;

@Config
@TeleOp
public class Caca extends LinearOpMode {

    SwerveDrivetrain drive;
    ElapsedTime timer;

    public static double time = 2;

    @Override
    public void runOpMode() throws InterruptedException {
        timer=new ElapsedTime();
        SwerveHardwareTest.init(hardwareMap);
        drive = new SwerveDrivetrain();

        waitForStart();

        timer.reset();

        while (opModeIsActive()){


            if(timer.seconds() <= time){
                drive.read();
                drive.set(new Pose(0, 0.5, 0));
                drive.write();
                drive.updateModules();
            }
            else{
                drive.read();
                drive.set(new Pose(0, 0, 0));
                drive.write();
                drive.updateModules();
            }

        }
    }
}
