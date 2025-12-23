package org.firstinspires.ftc.teamcode.htech.shooter;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.blob.localization.Odometry;
import org.firstinspires.ftc.teamcode.htech.config.SwerveHardwareTest;

@TeleOp(name = "Turret Odometry Tracker Test")
@Config
public class TurretOdometryTest extends LinearOpMode {

    public static double targetX = -118;
    public static double targetY = 20.5;

    @Override
    public void runOpMode() throws InterruptedException {
        SwerveHardwareTest.init(hardwareMap);

        Turret turret = new Turret(hardwareMap);
        turret.setVoltageSensor(SwerveHardwareTest.vs);
        Turret.voltageCompensation = true;

        Odometry odometry = new Odometry(SwerveHardwareTest.vs, SwerveHardwareTest.odo);
        TurretOdometryTracker tracker = new TurretOdometryTracker(odometry, turret, SwerveHardwareTest.vs);

        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        waitForStart();

        while (opModeIsActive()) {
            tracker.setTarget(targetX, targetY);
            tracker.update();

            telemetry.addData("Robot X", odometry.getX());
            telemetry.addData("Robot Y", odometry.getY());
            telemetry.addData("Robot Heading (deg)", Math.toDegrees(odometry.getHeading()));
            telemetry.addData("Turret Target Angle (deg)", Math.toDegrees(tracker.getTurretAngle(targetX, targetY)));
            telemetry.addData("Turret Target Ticks", turret.targetPosition);
            telemetry.addData("Turret Current Ticks", turret.currentPosition);
            telemetry.addData("Aligned", tracker.isAligned());
            telemetry.update();
        }
    }
}