package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;


import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.opencv.core.Mat;

@Config
public class SwerveModule {

    Servo servo;
    DcMotorEx motor;
    AnalogInput ai;

    public  double initialPosition=0; ///PWM cand rpata e la 0 grade

    public static double P = 0.15, I = 0.023, D = 0.1;
    public static double K_STATIC = 0.03;

    PIDFController controller=new PIDFController(P , I , D , 0);

    public double distance=0;

    public static double angle=0;
    private int reverse=1;

    private Vector vector=new Vector(0 ,0);

    public SwerveModule(DcMotorEx motor, Servo servo, AnalogInput ai , double initialPosition , boolean reverse)
    {
        this.motor=motor;
        this.servo=servo;
        this.ai=ai;
        this.initialPosition=initialPosition;
        if(reverse)motor.setDirection(DcMotorSimple.Direction.REVERSE);
    }

    public void setTargetVector(Vector vector)
    {
        this.vector=vector;

    }


    private void updateTargetPosition()
    {
        double angle=vector.getAngle(); //vector.getAngle()

        double normalError =  (angle+ java.lang.Math.PI*2)% (java.lang.Math.PI*2) - Math.PWMtoRADIANS( (ai.getVoltage()-initialPosition+3.3)%3.3 );
        if(java.lang.Math.abs(normalError)> java.lang.Math.PI )normalError = - java.lang.Math.signum (normalError ) * ( 2 * java.lang.Math.PI- java.lang.Math.abs(normalError));

        double mirrorError = (angle+ java.lang.Math.PI+ java.lang.Math.PI*2) % (java.lang.Math.PI*2) - Math.PWMtoRADIANS((ai.getVoltage()-initialPosition+3.3)%3.3);
        if(java.lang.Math.abs(mirrorError) > java.lang.Math.PI )mirrorError = -java.lang.Math.signum(mirrorError) * ( 2 * java.lang.Math.PI - java.lang.Math.abs(mirrorError));

        if (java.lang.Math.abs(normalError) < java.lang.Math.abs(mirrorError)) {distance=normalError; reverse=1;}
        else {distance=mirrorError; reverse=-1;}
    }

    private void updateMotor()
    {
        motor.setPower(reverse* (vector.getDistance()) );
    }

    private void updateServo()
    {
        double power=controller.calculate(0 , distance);
        servo.setPosition(0.5+power);
    }

    public void update(Telemetry telemetry)
    {
        controller.setPIDF(P, I, D, 0);


        updateTargetPosition();
        updateMotor();
        updateServo();


        //telemetry.addData("ai" , ai.getVoltage());
        telemetry.addData("targetAngle" , angle );
        telemetry.addData("servoAngle"+servo.getDeviceName() , Math.PWMtoRADIANS((ai.getVoltage()-initialPosition+3.3)%3.3   ));
        telemetry.addData( "ErrorDistance" , distance);

        //telemetry.addData("revese" , reverse);
        //telemetry.addData("servoPower" , servo.getPosition());


    }
}