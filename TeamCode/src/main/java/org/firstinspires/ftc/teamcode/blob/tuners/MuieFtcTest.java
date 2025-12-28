package org.firstinspires.ftc.teamcode.blob.tuners;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.blob.driveTrain.MuieFtc;
import org.firstinspires.ftc.teamcode.htech.utils.Pose;

@Config
@Autonomous
public class MuieFtcTest extends LinearOpMode {

    MuieFtc blob;

    public static double x = 0, y = 40, h = 0;

    boolean firstTime = true;

    @Override
    public void runOpMode() throws InterruptedException {

        blob = new MuieFtc(hardwareMap);
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        while (opModeInInit()){
            blob.init();
        }

        waitForStart();

        while (opModeIsActive()){

            if(firstTime){
                blob.setTargetPose(new Pose(x, y, h));
                firstTime = false;
            }
            else{
                if(blob.inPosition()){
                    blob.setTargetPose(new Pose(0, 0, 0));
                }
            }

            blob.update();
            telemetry.addData("odo x", blob.odo.getX());
            telemetry.addData("odo y", blob.odo.getY());
            telemetry.addData("odo h", blob.odo.getHeading());
            telemetry.addData("in Pos", blob.inPosition());
            telemetry.update();


        }

    }
}
