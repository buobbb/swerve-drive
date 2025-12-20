package org.firstinspires.ftc.teamcode.htech.swerve;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.htech.config.SwerveHardware;
import org.firstinspires.ftc.teamcode.htech.config.SwerveHardwareTest;

@Config
@TeleOp
public class SwerveTest extends LinearOpMode {

    private SwerveDrivetrain drive;

    @Override
    public void runOpMode() {
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        SwerveHardware.auto = false;
        SwerveHardwareTest.init(hardwareMap);
        drive = new SwerveDrivetrain();

        waitForStart();

        while (opModeIsActive()) {
            drive.updateMovement(gamepad1);
            telemetry.addLine(drive.getTelemetry());
            telemetry.update();
        }
    }
}
