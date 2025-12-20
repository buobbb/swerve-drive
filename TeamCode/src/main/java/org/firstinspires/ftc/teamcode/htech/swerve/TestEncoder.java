package org.firstinspires.ftc.teamcode.htech.swerve;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.AnalogInput;



@Config
@TeleOp
public class TestEncoder extends LinearOpMode {

    private AbsoluteAnalogEncoder flEnc;

    // aici memorăm offset-urile măsurate când apeși butoanele
    private double flOffset = 0.0;

    // pentru a nu reciti de 1000 ori pe apăsare
    private boolean lastA = false;

    public static boolean invertFl = false;

    public static AnalogInput frontLeftEncoder;
    public static String port = "";



    @Override
    public void runOpMode() throws InterruptedException {

        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        frontLeftEncoder = hardwareMap.get(AnalogInput.class, port);

        // ATENȚIE: aici pui .setInverted(true/false) EXACT cum vrei să fie în codul final
        flEnc = new AbsoluteAnalogEncoder(frontLeftEncoder, 3.3)
                .setInverted(invertFl);

        telemetry.addLine("Swerve encoder calib");
        telemetry.addLine("A = salveaza offset FL");
        telemetry.addLine("B = salveaza offset FR");
        telemetry.addLine("X = salveaza offset BL");
        telemetry.addLine("Y = salveaza offset BR");
        telemetry.addLine("PUNE TOATE ROTILE PERFECT DREPTE INAINTE SA APESI");
        telemetry.update();

        waitForStart();

        while (opModeIsActive()) {

            // citim unghiurile actuale (fara offset, pentru ca implicit offset = 0)
            double flAngle = flEnc.getCurrentPosition();

            // când apeși A → memorăm offset pentru front left
            if (gamepad1.a && !lastA) {
                flOffset = flAngle;
            }


            lastA = gamepad1.a;

            telemetry.addLine("---- LIVE ANGLES (radiani) ----");
            telemetry.addData("FL angle", flAngle);

            telemetry.addLine("---- OFFSETURI MEMORATE ----");
            telemetry.addData("frontLeftOffset  (-> pune in SwerveDrivetrain)", flOffset);


            telemetry.update();
        }
    }
}
