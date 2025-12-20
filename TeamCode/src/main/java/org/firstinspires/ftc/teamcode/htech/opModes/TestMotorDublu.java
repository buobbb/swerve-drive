package org.firstinspires.ftc.teamcode.htech.opModes;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

@Config
@TeleOp
public class TestMotorDublu extends LinearOpMode {

    DcMotorEx m1, m2;

    public static String m1Port = "";
    public static String m2Port = "";

    public static double m1Power = 0;
    public static double m2Power = 0;

    public static boolean m1Reverse = false;
    public static boolean m2Reverse = true;

    public static boolean samePower = true;
    public static double powerForBoth = 0;


    @Override
    public void runOpMode() throws InterruptedException {
        m1 = hardwareMap.get(DcMotorEx.class, m1Port);
        m2 = hardwareMap.get(DcMotorEx.class, m2Port);

        m1.setDirection(m1Reverse ? DcMotorSimple.Direction.REVERSE : DcMotorSimple.Direction.FORWARD);
        m2.setDirection(m2Reverse ? DcMotorSimple.Direction.REVERSE : DcMotorSimple.Direction.FORWARD);

        waitForStart();

        while (opModeIsActive()){


            if(samePower) {
                m1.setPower(powerForBoth);
                m2.setPower(powerForBoth);
            } else {
                m1.setPower(m1Power);
                m2.setPower(m2Power);
            }


        }
    }
}
