package org.firstinspires.ftc.teamcode.htech.opModes;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;

@Config
@TeleOp
public class TestServo extends LinearOpMode {

    Servo s;
    public static String servoName = "";
    public static double position = 0.5;

    @Override
    public void runOpMode() throws InterruptedException {
        s = hardwareMap.get(Servo.class, servoName);

        waitForStart();

        while (opModeIsActive()) {
            s.setPosition(position);
        }
    }
}
