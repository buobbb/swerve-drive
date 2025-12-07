package org.firstinspires.ftc.teamcode.htech.opModes;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorEx;

@Config
@TeleOp
public class TestMotor extends LinearOpMode {

    DcMotorEx m;
    public static String port = "";
    public static double power = 0;

    @Override
    public void runOpMode() throws InterruptedException {
        m = hardwareMap.get(DcMotorEx.class, port);

        waitForStart();

        while (opModeIsActive()) {
            m.setPower(power);
        }
    }
}
