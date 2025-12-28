package org.firstinspires.ftc.teamcode.htech.opModes;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;

import org.firstinspires.ftc.teamcode.htech.config.mechanisms_hardware.Motors;
import org.firstinspires.ftc.teamcode.htech.shooter.ShooterMath;

@Config
@TeleOp
public class TestTurret extends LinearOpMode {

    DcMotorEx m;
    ShooterMath math;
    DcMotorEx encoder;

    public static String encoderPort = "m1";
    public static boolean reverseMotor = false;

    public static double angle = 0;

    @Override
    public void runOpMode() throws InterruptedException {
        m = hardwareMap.get(DcMotorEx.class, Motors.turretMotor);
        encoder = hardwareMap.get(DcMotorEx.class, encoderPort);
        math = new ShooterMath();
        m.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        m.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        encoder.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        encoder.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        m.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        m.setDirection(reverseMotor ? DcMotor.Direction.REVERSE : DcMotor.Direction.FORWARD);
        encoder.setDirection(reverseMotor ? DcMotor.Direction.REVERSE : DcMotor.Direction.FORWARD);
        telemetry = new MultipleTelemetry(telemetry, com.acmerobotics.dashboard.FtcDashboard.getInstance().getTelemetry());
        waitForStart();

        while (opModeIsActive()){

            telemetry.addData("pos", encoder.getCurrentPosition());
            telemetry.addData("angle to ticks" , math.angleToTicks(angle));
            telemetry.update();
        }
    }
}
