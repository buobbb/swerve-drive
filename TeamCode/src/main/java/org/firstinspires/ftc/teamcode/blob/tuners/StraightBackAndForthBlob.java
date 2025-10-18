package org.firstinspires.ftc.teamcode.blob.tuners;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.blob.driveTrain.BlobSwerve;
import org.firstinspires.ftc.teamcode.htech.config.SwerveHardware;
import org.firstinspires.ftc.teamcode.htech.swerve.SwerveDrivetrain;

@Config
@Autonomous
public class StraightBackAndForthBlob extends LinearOpMode {

    BlobSwerve blob;
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
        SwerveHardware.init(hardwareMap);
        drive = new SwerveDrivetrain();
        blob = new BlobSwerve(drive);
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        waitForStart();

        cs = STATES.FORWARD;
        firstTime = true;

        while (opModeIsActive()){

            switch (cs){

                case FORWARD:
                    if(firstTime) {
                        blob.setTargetPosition(fx, fy, Math.toRadians(fh));
                        firstTime = false;
                    }
                    else if(blob.inPosition()) {
                        cs = STATES.BACKWARDS;
                        firstTime = true;
                    }
                    break;

                case BACKWARDS:
                    if(firstTime) {
                        blob.setTargetPosition(bx, by, bh);
                        firstTime = false;
                    }
                    else if(blob.inPosition()) {
                        cs = STATES.FORWARD;
                        firstTime = true;
                    }
                    break;

                case IDLE:
                    break;

            }

            telemetry.addData("x", blob.odo.getX());
            telemetry.addData("y", blob.odo.getY());
            telemetry.addData("heading", blob.odo.getHeading());
            telemetry.addData("target Heading", blob.targetHeading);
            telemetry.addData("real Heading", blob.realHeading);

            blob.update();
            telemetry.update();
        }
    }
}
