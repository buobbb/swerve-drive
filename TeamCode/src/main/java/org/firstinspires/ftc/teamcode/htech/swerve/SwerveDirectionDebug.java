package org.firstinspires.ftc.teamcode.htech.swerve;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.htech.config.SwerveHardware;
import org.firstinspires.ftc.teamcode.htech.config.SwerveHardwareTest;

@TeleOp(name = "Swerve Direction Debug", group = "Swerve")
public class SwerveDirectionDebug extends LinearOpMode {

    SwerveDrivetrain drive;

    @Override
    public void runOpMode() throws InterruptedException {
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        SwerveHardwareTest.init(hardwareMap);
        drive = new SwerveDrivetrain();

        waitForStart();

        while (opModeIsActive()) {
            drive.read();

            // exemplu: impinge pe dpad
            double x = 0, y = 0, w = 0;
            if (gamepad1.dpad_up)    y =  1;
            if (gamepad1.dpad_down)  y = -1;
            if (gamepad1.dpad_right) x =  1;
            if (gamepad1.dpad_left)  x = -1;

            drive.set(new org.firstinspires.ftc.teamcode.htech.utils.Pose(x, y, w));
            drive.write();
            drive.updateModules();

            telemetry.addLine(drive.getTelemetry());
            telemetry.update();
        }
    }
}

