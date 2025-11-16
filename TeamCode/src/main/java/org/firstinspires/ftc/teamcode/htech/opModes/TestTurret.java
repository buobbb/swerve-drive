package org.firstinspires.ftc.teamcode.htech.opModes;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorEx;

@Config
@TeleOp
public class TestTurret extends LinearOpMode {

    DcMotorEx m;

    @Override
    public void runOpMode() throws InterruptedException {
        m = hardwareMap.get(DcMotorEx.class, "m0");

        waitForStart();

        while (opModeIsActive()){

            m.setPower(gamepad1.right_trigger - gamepad1.left_trigger);

        }
    }
}
