package org.firstinspires.ftc.teamcode.htech.opModes;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorEx;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

@Config
@TeleOp
public class TestMotor extends LinearOpMode {

    DcMotorEx m;
    public static String port = "";
    public static double power = 0;
    public static double vel = 0;

    @Override
    public void runOpMode() throws InterruptedException {
        m = hardwareMap.get(DcMotorEx.class, port);
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        waitForStart();

        while (opModeIsActive()) {
            if(gamepad1.right_bumper) m.setVelocity(vel);
            if(gamepad1.left_bumper) m.setPower(power);

            telemetry.addData("power", m.getPower());
            telemetry.addData("vel", m.getVelocity());
            telemetry.update();
        }
    }
}
