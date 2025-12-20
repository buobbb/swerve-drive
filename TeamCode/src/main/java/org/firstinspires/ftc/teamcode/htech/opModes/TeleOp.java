package org.firstinspires.ftc.teamcode.htech.opModes;

import static org.firstinspires.ftc.robotcore.external.navigation.AngleUnit.normalizeRadians;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.arcrobotics.ftclib.controller.PIDFController;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.htech.config.SwerveHardwareTest;
import org.firstinspires.ftc.teamcode.htech.robot.RobotSystems;
import org.firstinspires.ftc.teamcode.htech.swerve.SwerveDrivetrain;

@Config
@com.qualcomm.robotcore.eventloop.opmode.TeleOp(name = "AA TeleOp")
public class TeleOp extends LinearOpMode {

    // Higher values (8-10) make the robot snappier; lower values (2-4) make it smoother.

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
            telemetry.addData("Target Heading", Math.toDegrees(targetHeading));
            telemetry.update();
            drivetrain.updateMovement(gamepad1);
//            drivetrain.updateMovementFieldCentric(gamepad1);
        }
    }

    private void handleSubsystems() {
        if (g1.b) {
            gamepad1.rumble(150);
            shootFar = !shootFar;
            r.turret.far = shootFar;
        }

        if (g1.left_bumper) {
            if (r.s_CS == RobotSystems.ShooterStates.IDLE) {
                r.s_CS = RobotSystems.ShooterStates.CHARGING_UP;
                if (shootFar) r.launcher.shootFar();
                else r.launcher.shootClose();
            } else {
                r.buttonPressed = true;
            }
        }

        if (g1.right_bumper) {
            r.s_CS = RobotSystems.ShooterStates.IDLE;
            r.turret.readLl = false;
            r.launcher.stop();
        }

        if (r.s_CS == RobotSystems.ShooterStates.IDLE) {
            r.intake.setPower(gamepad1.right_trigger > 0.1 ? 1 : gamepad1.left_trigger > 0.1 ? -1 : 0);
//            r.intake.setPower(gamepad1.right_trigger - gamepad1.left_trigger);
            r.launcher.setPower(gamepad1.right_trigger > 0.1 ? -0.3 : 0);
        }

        if (r.rumbling) {
            r.rumbling = false;
            gamepad1.rumble(100);
        }
    }
}