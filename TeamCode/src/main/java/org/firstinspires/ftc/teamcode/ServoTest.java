package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@Config
@TeleOp
public class ServoTest extends LinearOpMode {

    enum State{
        PORT0 , PORT1 , PORT2 , PORT3, PORT4, PORT5;
    }
    public static State state=State.PORT0;
    public static double power;
    @Override
    public void runOpMode() throws InterruptedException {

        Hardware.init(hardwareMap);
        waitForStart();

        while(opModeIsActive())
        {
            if(state== State.PORT0)Hardware.s0se.setPosition(power);
            if(state== State.PORT1)Hardware.s1se.setPosition(power);
            if(state== State.PORT2)Hardware.s2se.setPosition(power);
            if(state== State.PORT3)Hardware.s3se.setPosition(power);
            if(state== State.PORT4)Hardware.s4se.setPosition(power);
            if(state== State.PORT5)Hardware.s5se.setPosition(power);



        }
    }
}
