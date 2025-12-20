//package org.firstinspires.ftc.teamcode.htech.shooter;
//
//import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
//import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
//import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
//
//import org.firstinspires.ftc.teamcode.htech.config.SwerveHardwareTest;
//import org.firstinspires.ftc.teamcode.htech.swerve.SwerveDrivetrain;
//
//@TeleOp
//public class testAutoPosShooter extends LinearOpMode {
//    TurretPidTx t;
//    SwerveDrivetrain drive;
//    @Override
//    public void runOpMode() throws InterruptedException {
//        SwerveHardwareTest.init(hardwareMap);
//        drive = new SwerveDrivetrain();
//        telemetry = new MultipleTelemetry(telemetry, com.acmerobotics.dashboard.FtcDashboard.getInstance().getTelemetry());
//        t = new TurretPidTx(hardwareMap);
//        waitForStart();
//        while(opModeIsActive()){
//            t.update();
//            drive.updateMovement(gamepad1);
//            telemetry.addData("current Pos", t.currentPosTx);
////            telemetry.addData("Tx", t.result.getTx());
//            telemetry.update();
//        }
//
//    }
//}
