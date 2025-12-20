package org.firstinspires.ftc.teamcode;


import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.IMU;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

@Config
@TeleOp
public class Test extends LinearOpMode {


    public static double multiplier=0.4;
    public static double P=0.3 , I=0.1 , D=0.5;
    public static double limitation=0.5;

    @Override
    public void runOpMode() throws InterruptedException {

        Hardware.init(hardwareMap);

        IMU imu;
        imu = hardwareMap.get(IMU.class, "imu");
        PIDFController controller=new PIDFController(P , I , D , 0);

        double targetAngle=0;


        IMU.Parameters parameters = new IMU.Parameters(new RevHubOrientationOnRobot(
                RevHubOrientationOnRobot.LogoFacingDirection.LEFT,
                RevHubOrientationOnRobot.UsbFacingDirection.FORWARD));

        imu.initialize(parameters);

        SwerveModule frontLeft= new SwerveModule( Hardware.m0, Hardware.s4se, Hardware.a0e, 1.652 , true );
        SwerveModule frontRight= new SwerveModule( Hardware.m2e, Hardware.s2se, Hardware.a2e, 2.832 , true );
        SwerveModule rearLeft= new SwerveModule( Hardware.m1, Hardware.s1se, Hardware.a1e, 0.72 , true );
        SwerveModule rearRight= new SwerveModule( Hardware.m3e, Hardware.s3se, Hardware.a3e, 2.740 , true );

        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());


        waitForStart();
        while(opModeIsActive())
        {
            if(gamepad1.a)limitation=1;
            else limitation=0.5;

            double Y = -gamepad1.left_stick_y*limitation;
            double X = gamepad1.left_stick_x*limitation;
            targetAngle+= (gamepad1.left_trigger - gamepad1.right_trigger)*multiplier;

            if (gamepad1.options) {
                imu.resetYaw();
            }



            double botHeading = imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS);

            double x = X * java.lang.Math.cos(-botHeading) - Y * java.lang.Math.sin(-botHeading);
            double y = X * java.lang.Math.sin(-botHeading) + Y * java.lang.Math.cos(-botHeading);


            double error =  (targetAngle+ java.lang.Math.PI*10)% (java.lang.Math.PI*2) - (botHeading+ java.lang.Math.PI*10)%(java.lang.Math.PI*2);

            if(java.lang.Math.abs(error)> java.lang.Math.PI )error = - java.lang.Math.signum (error ) * ( 2 * java.lang.Math.PI- java.lang.Math.abs(error));


            double power=controller.calculate(0 , error);

            double v = java.lang.Math.abs(x) + java.lang.Math.abs(power);
            double v1 = java.lang.Math.abs(y) + java.lang.Math.abs(power);
            double maxDistance= java.lang.Math.sqrt( v * v + v1 * v1);

            double denominator = java.lang.Math.max(1 , maxDistance      );

            frontLeft.setTargetVector( new Vector ((x - power)/denominator , (y - power)/denominator ));
            frontRight.setTargetVector( new Vector ( (x - power)/denominator , (y + power )/denominator ));
            rearLeft.setTargetVector( new Vector( (x + power)/denominator , (y - power )/denominator ));
            rearRight.setTargetVector( new Vector( (x + power)/denominator , (y + power )/denominator ));

            controller.setPIDF(P , I , D , 0);

            frontLeft.update(telemetry);
            frontRight.update(telemetry);
            rearLeft.update(telemetry);
            rearRight.update(telemetry);


            telemetry.addData("targetAngle" , SwerveModule.angle );
            telemetry.addData( "ErrorDistance" , frontLeft.distance);
            telemetry.addData("servoAngle1" , Math.PWMtoRADIANS((frontLeft.ai.getVoltage()- frontLeft.initialPosition+3.3)%3.3   ));
            telemetry.addData("servoAngle2" , Math.PWMtoRADIANS((frontRight.ai.getVoltage()- frontRight.initialPosition+3.3)%3.3   ));
            telemetry.addData("servoAngle3" , Math.PWMtoRADIANS((rearLeft.ai.getVoltage()- rearLeft.initialPosition+3.3)%3.3   ));
            telemetry.addData("servoAngle4" , Math.PWMtoRADIANS((rearRight.ai.getVoltage()- rearRight.initialPosition+3.3)%3.3   ));
            telemetry.addData("IMU" , imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS));

            telemetry.update();
        }
    }
}
