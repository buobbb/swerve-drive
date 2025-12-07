package org.firstinspires.ftc.teamcode.htech.opModes;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;

@Config
@TeleOp
public class TestCrServo extends LinearOpMode {

    CRServo s;
    public static String port = "";
    public static double power = 0;

    @Override
    public void runOpMode() throws InterruptedException {
        s = hardwareMap.get(CRServo.class, port);

        waitForStart();

        while (opModeIsActive()) {
            s.setPower(power);
        }
    }
}
