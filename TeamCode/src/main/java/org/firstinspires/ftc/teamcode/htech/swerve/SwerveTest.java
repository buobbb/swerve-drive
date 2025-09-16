package org.firstinspires.ftc.teamcode.htech.swerve;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.htech.config.Hardware;

@Config
@TeleOp
public class SwerveTest extends LinearOpMode {

    private SwerveDrivetrain drive;

    @Override
    public void runOpMode() {
        Hardware.auto = false;
        Hardware.init(hardwareMap);
        drive = new SwerveDrivetrain();

        waitForStart();

        while (opModeIsActive()) {
            drive.updateMovement(gamepad1);
            telemetry.addLine(drive.getTelemetry());
            telemetry.update();
        }
    }
}
