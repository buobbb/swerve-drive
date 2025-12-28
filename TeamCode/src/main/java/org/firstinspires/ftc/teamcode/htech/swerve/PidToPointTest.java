package org.firstinspires.ftc.teamcode.htech.swerve;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.blob.localization.Odometry;
import org.firstinspires.ftc.teamcode.htech.config.SwerveHardwareTest;
import org.firstinspires.ftc.teamcode.htech.utils.Pose;

@Config
@Autonomous(name = "PidToPointTest", group = "swerve")
public class PidToPointTest extends LinearOpMode {
    public static double startX = 0;
    public static double startY = 0;
    public static double startH = 0;

    public static double targetX = 36;
    public static double targetY = 0;
    public static double targetH = 0;

    enum STATES{
        FORWARDS,
        BACKWARDS,
        RIGHT,
        IDLE
    }
    STATES CS = STATES.IDLE;
    boolean firstTime;

    private PidToPoint pidToPoint;
    private Odometry odometry;
    private SwerveDrivetrain drivetrain;

    @Override
    public void runOpMode() throws InterruptedException {
        SwerveHardwareTest.init(hardwareMap);
        drivetrain = new SwerveDrivetrain();
        odometry = new Odometry(SwerveHardwareTest.vs, SwerveHardwareTest.odo);
        pidToPoint = new PidToPoint(drivetrain, odometry);
        pidToPoint.setVoltageSensor(SwerveHardwareTest.vs);

        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        Pose startPose = new Pose(startX, startY, startH);
        pidToPoint.setTargetPose(new Pose(targetX, targetY, targetH));

        while (opModeInInit()) {
            pidToPoint.init(startPose);
            telemetry.addData("init pose", startPose);
            telemetry.update();
        }

        waitForStart();

        CS = STATES.FORWARDS;
        firstTime = true;

        while (opModeIsActive()) {

            switch (CS){

                case FORWARDS:
                    if(firstTime){
                        pidToPoint.setTargetPose(new Pose(targetX, targetY, targetH));
                        firstTime = false;
                    }
                    else{
                        if(pidToPoint.inPosition()){
                            CS = STATES.IDLE;
                            firstTime = true;
                        }
                    }
                    break;

                case BACKWARDS:
                    if(firstTime){
                        pidToPoint.setTargetPose(new Pose(startX, startY, startH));
                        firstTime = false;
                    }
                    else{
                        if(pidToPoint.inPosition()){
                            CS = STATES.FORWARDS;
                            firstTime = true;
                        }
                    }
                    break;

                case IDLE:
                    break;

            }

            pidToPoint.update();

            telemetry.addData("x", odometry.getX());
            telemetry.addData("y", odometry.getY());
            telemetry.addData("heading", odometry.getHeading());
            telemetry.addData("target", targetX + ", " + targetY + ", " + targetH);
            telemetry.addData("in position", pidToPoint.inPosition());
            telemetry.addData("rotation power", pidToPoint.robotHeading);
            telemetry.update();
        }
    }
}