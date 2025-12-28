package org.firstinspires.ftc.teamcode.htech.opModes;

import static org.firstinspires.ftc.robotcore.external.navigation.AngleUnit.normalizeRadians;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.arcrobotics.ftclib.controller.PIDFController;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.htech.config.SwerveHardwareTest;
import org.firstinspires.ftc.teamcode.htech.robot.RobotSystems;
import org.firstinspires.ftc.teamcode.htech.shooter.Launcher;
import org.firstinspires.ftc.teamcode.htech.swerve.SwerveDrivetrain;

import java.util.List;

@Config
@com.qualcomm.robotcore.eventloop.opmode.TeleOp(name = "AA TeleOp")
public class TeleOp extends LinearOpMode {

    // Higher values (8-10) make the robot snappier; lower values (2-4) make it smoother.

    List<LynxModule> allHubs;
    SwerveDrivetrain drivetrain;
    RobotSystems r;
    htech.utils.StickyGamepad g1;

    // Heading PID: Adjusted P to be more aggressive for Snapping
    private final PIDFController hController = new PIDFController(1.5, 0, 0.1, 0);

    boolean lock_robot_heading = false;
    boolean shootFar = false;
    double targetHeading = 0;

    @Override
    public void runOpMode() throws InterruptedException {
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        allHubs = hardwareMap.getAll(LynxModule.class);

        for (LynxModule hub : allHubs) {
            hub.setBulkCachingMode(LynxModule.BulkCachingMode.MANUAL);
        }

        SwerveHardwareTest.init(hardwareMap);
        drivetrain = new SwerveDrivetrain();
        r = new RobotSystems(hardwareMap);
        g1 = new htech.utils.StickyGamepad(gamepad1, this);

        waitForStart();

        while (opModeIsActive()) {
            handleSubsystems();

            g1.update();
            r.update();

            // Telemetry
            telemetry.addData("s_CS", r.s_CS);
            telemetry.addData("rt_CS", r.rt_cs);
            telemetry.addData("Robot X", r.odo.getX());
            telemetry.addData("Robot Y", r.odo.getY());
            telemetry.addData("Robot Heading (deg)", Math.toDegrees(r.odo.getHeading()));
            telemetry.addData("Turret Target Angle (deg)", Math.toDegrees(r.tracker.getTurretAngle(RobotSystems.targetX, RobotSystems.targetY)));
            telemetry.addData("Turret Target Ticks", r.turret.targetPosition);
            telemetry.addData("Turret Current Ticks", r.turret.currentPosition);
            telemetry.addData("Aligned", r.tracker.isAligned());
            telemetry.addData("curr vel", r.launcher.currentVelocity);
            telemetry.addData("target vel", Launcher.targetVelocityClose);
            telemetry.addData("distance", r.distance);
            telemetry.update();
            drivetrain.updateMovement(gamepad1);

            for (LynxModule hub : allHubs) {
                hub.clearBulkCache();
            }

//            drivetrain.updateMovementFieldCentric(gamepad1);
        }
    }

    private void handleSubsystems() {
        if (g1.b) {
            gamepad1.rumble(150);
            shootFar = !shootFar;
        }

        if (g1.left_bumper) {
            r.shoot();
            if (shootFar) r.launcher.shootFar();
            else r.launcher.shootClose();
        }

        if (g1.right_bumper) {
            r.s_CS = RobotSystems.ShooterStates.IDLE;
            r.launcher.stop();
        }

        if(g1.right_stick_button){
            r.resetTurret();
        }

        if(g1.a){
            if(lock_robot_heading){
                lock_robot_heading = false;
                drivetrain.setLocked(false);
            }
            else{
                lock_robot_heading = true;
                drivetrain.setLocked(true);
            }
        }

        if (r.s_CS == RobotSystems.ShooterStates.IDLE) {
            r.intake.setPower(gamepad1.right_trigger > 0.1 ? 1 : gamepad1.left_trigger > 0.1 ? -1 : 0);
//            r.intake.setPower(gamepad1.right_trigger - gamepad1.left_trigger);
            r.launcher.setPower(gamepad1.right_trigger > 0.1 ? -0.356 : 0);
        }

        if (r.rumbling) {
            r.rumbling = false;
            gamepad1.rumble(100);
        }
    }
}