package org.firstinspires.ftc.teamcode.blob.tuners;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.blob.driveTrain.BlobCoaieMari;
import org.firstinspires.ftc.teamcode.blob.driveTrain.BlobSwerve;
import org.firstinspires.ftc.teamcode.blob.driveTrain.LucaSwerve;
import org.firstinspires.ftc.teamcode.blob.localization.Odometry;
import org.firstinspires.ftc.teamcode.htech.config.SwerveHardware;
import org.firstinspires.ftc.teamcode.htech.config.SwerveHardwareTest;
import org.firstinspires.ftc.teamcode.htech.swerve.SwerveDrivetrain;
import org.firstinspires.ftc.teamcode.htech.utils.Pose;

@Config
@Autonomous
public class StraightBackAndForthBlob extends LinearOpMode {

    LucaSwerve lucaSwerve;
    Odometry localizer;
    SwerveDrivetrain drive;

    public static double fx = 39.37, fy = 0, fh = 0;
    public static double bx = 0, by = 0, bh = 0;
    public static double rx = 0, ry = -20, rh = Math.toRadians(270);

    enum STATES{
        FORWARD,
        BACKWARDS,
        RIGHT,
        IDLE
    }
    STATES cs =  STATES.IDLE;

    boolean firstTime;

    @Override
    public void runOpMode() throws InterruptedException {
        SwerveHardwareTest.init(hardwareMap);
        drive = new SwerveDrivetrain();
        localizer = new Odometry(SwerveHardwareTest.vs, SwerveHardwareTest.odo);
        lucaSwerve = new LucaSwerve(drive, localizer, new Pose(0,0,0));
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        while(opModeInInit()) {
            lucaSwerve.init();
        }

        cs = STATES.FORWARD;
        firstTime = true;

        while (opModeIsActive()){

            switch (cs){

                case FORWARD:
                    if(firstTime) {
                        lucaSwerve.setTargetPose(new Pose(fx, fy, 0));
                        firstTime = false;
                    }
                    else if(lucaSwerve.inPosition()) {
                        cs = STATES.BACKWARDS;
                        firstTime = true;
                    }
                    break;

                case BACKWARDS:
                    if(firstTime) {
                        lucaSwerve.setTargetPose(new Pose(bx, by, 0));
                        firstTime = false;
                    }
                    else if(lucaSwerve.inPosition()) {
                        cs = STATES.FORWARD;
                        firstTime = true;
                    }
                    break;

                case IDLE:
                    break;

            }

            telemetry.addData("x", lucaSwerve.localizer.getX());
            telemetry.addData("y", lucaSwerve.localizer.getY());
            telemetry.addData("heading", lucaSwerve.localizer.getHeading());
            telemetry.addData("inPos", lucaSwerve.inPosition());
            telemetry.addData("CS", cs);
            lucaSwerve.update();
            telemetry.update();
        }
    }
}
