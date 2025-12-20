package org.firstinspires.ftc.teamcode.htech.opModes;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.htech.config.SwerveHardwareTest;
import org.firstinspires.ftc.teamcode.htech.robot.RobotSystemsX2;
import org.firstinspires.ftc.teamcode.htech.shooter.Shooter;
import org.firstinspires.ftc.teamcode.htech.swerve.SwerveDrivetrain;

@Config
@TeleOp
public class TestTurelaOdometrie extends LinearOpMode {

    RobotSystemsX2 r;
    SwerveDrivetrain drive;

    @Override
    public void runOpMode() throws InterruptedException {
        SwerveHardwareTest.init(hardwareMap);
        drive = new SwerveDrivetrain();
        telemetry = new MultipleTelemetry(telemetry, com.acmerobotics.dashboard.FtcDashboard.getInstance().getTelemetry());

        r = new RobotSystemsX2(hardwareMap);

        waitForStart();

        while (opModeIsActive()){

            drive.updateMovement(gamepad1);
            r.update();

            telemetry.addData("distance to goal", r.shooter.distance);
            telemetry.addData("robotX", r.shooter.robotX);
            telemetry.addData("robotY", r.shooter.robotY);
            telemetry.addData("robotH", r.shooter.robotH);
            telemetry.addData("turret angle", Math.toRadians(r.shooter.turretAngle));
            telemetry.addData("turret angle to ticks", r.shooter.math.angleToTicks(r.shooter.turretAngle));
            telemetry.update();
        }

    }
}
