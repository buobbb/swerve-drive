package org.firstinspires.ftc.teamcode.htech.swerve;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.htech.config.SwerveHardwareTest;

@Config
@TeleOp
public class MuieLucaPredoi extends LinearOpMode {

    SwerveModuleKooky module;
    public static double target = 0;
    public static double offset = 3.87;


    @Override
    public void runOpMode() throws InterruptedException {
        module  = new SwerveModuleKooky(SwerveHardwareTest.frontLeftMotor,  SwerveHardwareTest.frontLeftServo,  new AbsoluteAnalogEncoder(SwerveHardwareTest.frontLeftEncoder,  3.3).zero(offset).setInverted(true));
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        while (opModeIsActive()){

            module.setTargetRotation(target);
            module.update();

            telemetry.addData("Current", module.getModuleRotation());
            telemetry.addData("Target", target);
            telemetry.update();

        }
    }
}
