package org.firstinspires.ftc.teamcode.htech.opModes;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;

import org.firstinspires.ftc.teamcode.htech.config.SwerveHardwareTest;
import org.firstinspires.ftc.teamcode.htech.swerve.SwerveDrivetrain;

@Config
@TeleOp
public class RobotLaMisto extends LinearOpMode {

    SwerveDrivetrain drive;
    DcMotorEx intakeMotor;
    DcMotorEx shooterMotor;
    DcMotorEx turretMotor;

    public static String intakeMotorName = "m0e";
    public static String shooterMotorName = "m1e";
    public static String turretMotorName = "m2";

    public static boolean reverseIntake = true;
    public static boolean reverseShooter = false;

    @Override
    public void runOpMode() throws InterruptedException {
        intakeMotor = hardwareMap.get(DcMotorEx.class, intakeMotorName);
        shooterMotor = hardwareMap.get(DcMotorEx.class, shooterMotorName);
        turretMotor = hardwareMap.get(DcMotorEx.class, turretMotorName);

        SwerveHardwareTest.init(hardwareMap);
        drive = new SwerveDrivetrain();

        intakeMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        shooterMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        turretMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        intakeMotor.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
        shooterMotor.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
        turretMotor.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);

        intakeMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        shooterMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        turretMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        intakeMotor.setDirection(reverseIntake ? DcMotor.Direction.REVERSE : DcMotor.Direction.FORWARD);
        shooterMotor.setDirection(reverseShooter ? DcMotor.Direction.REVERSE : DcMotor.Direction.FORWARD);

        waitForStart();

        while(opModeIsActive()){

            shooterMotor.setPower(gamepad1.right_trigger - gamepad1.left_trigger);
            intakeMotor.setPower(gamepad1.right_bumper ? 1 : gamepad1.left_bumper ? -1 : 0);

            drive.updateMovement(gamepad1);
        }


    }
}
