package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@Config
@TeleOp
public class AnalogInputTest extends LinearOpMode {


    @Override
    public void runOpMode() throws InterruptedException {
        Hardware.init(hardwareMap);
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        waitForStart();

        while(opModeIsActive())
        {
            Hardware.s0se.setPosition(0.5);
            Hardware.s1se.setPosition(0.5);
            Hardware.s2se.setPosition(0.5);
            Hardware.s3se.setPosition(0.5);


            telemetry.addData("ai0" , Hardware.a0e.getVoltage());
            telemetry.addData("ai0_MAXVOLTAGE" , Hardware.a0e.getMaxVoltage());

            telemetry.addData("ai1" , Hardware.a1e.getVoltage());
            telemetry.addData("ai1_MAXVOLTAGE" , Hardware.a1e.getMaxVoltage());

            telemetry.addData("ai2" , Hardware.a2e.getVoltage());
            telemetry.addData("ai2_MAXVOLTAGE" , Hardware.a2e.getMaxVoltage());

            telemetry.addData("ai3" , Hardware.a3e.getVoltage());
            telemetry.addData("ai3_MAXVOLTAGE" , Hardware.a3e.getMaxVoltage());

            telemetry.update();
        }

    }
}
