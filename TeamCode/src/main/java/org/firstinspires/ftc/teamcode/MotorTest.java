package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@Config
@TeleOp
public class MotorTest extends LinearOpMode {

    enum State{
        PORT0 , PORT1 , PORT2 , PORT3;
    }
    public static State state=State.PORT0;
    public static double power;
    @Override
    public void runOpMode() throws InterruptedException {

        Hardware.init(hardwareMap);
        waitForStart();

        while(opModeIsActive())
        {
            if(state==State.PORT0)Hardware.m0.setPower(power);
            if(state==State.PORT1)Hardware.m1.setPower(power);
            if(state==State.PORT2)Hardware.m2e.setPower(power);
            if(state==State.PORT3)Hardware.m3e.setPower(power);

        }
    }
}
