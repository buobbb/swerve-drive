package org.firstinspires.ftc.teamcode.htech.shooter;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.blob.localization.Odometry;
import org.firstinspires.ftc.teamcode.htech.config.SwerveHardwareTest;
import org.firstinspires.ftc.teamcode.htech.robot.RobotSystems;
import org.firstinspires.ftc.teamcode.htech.swerve.MathUtils;
import org.firstinspires.ftc.teamcode.htech.swerve.SwerveDrivetrain;

@TeleOp
@Config
public class TurretFollowBabi extends LinearOpMode {

    public ShooterMath math;

    SwerveDrivetrain drive;

    Turret turret;
    Odometry odo;
    @Override
    public void runOpMode() throws InterruptedException {

        SwerveHardwareTest.init(hardwareMap);
        drive = new SwerveDrivetrain();

        math = new ShooterMath();
        turret = new Turret(hardwareMap);

        odo = new Odometry(SwerveHardwareTest.vs, SwerveHardwareTest.odo);
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());



        waitForStart();
        while(opModeIsActive()){

            double heading = Math.toDegrees(odo.getHeading());
            heading = heading % 360;
            heading = (heading + 360) % 360;

            if(heading > 180) heading -= 360;

            double angle = MathUtils.clamp(heading, -90, 90);

            turret.setPosition(-math.angleToTicks(Math.toRadians(angle)));
            turret.update();
            odo.update();

            telemetry.addData("Angle in Degrees", angle);
            telemetry.addData("heading", Math.toDegrees(odo.getHeading()));
            telemetry.addData("Turret Target Pos", turret.targetPosition);
            telemetry.addData("Turret Current Pos", turret.currentPosition);


            drive.updateMovement(gamepad1);
            telemetry.update();

        }

    }
}
