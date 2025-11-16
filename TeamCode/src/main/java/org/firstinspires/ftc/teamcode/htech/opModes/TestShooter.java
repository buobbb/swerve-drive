package org.firstinspires.ftc.teamcode.htech.opModes;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.htech.shooter.Launcher;

@Config
@TeleOp
public class TestShooter extends LinearOpMode {

    Launcher launcher;


    @Override
    public void runOpMode() throws InterruptedException {
        launcher = new Launcher(hardwareMap);

        waitForStart();

        while (opModeIsActive()){

            launcher.setPowerMuie(gamepad1.right_trigger - gamepad1.left_trigger);

            launcher.update();
        }
    }
}
