package org.firstinspires.ftc.teamcode.htech.swerve;

import static java.lang.Math.atan2;
import static java.lang.Math.hypot;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Gamepad;

import org.firstinspires.ftc.teamcode.blob.localization.Odometry;
import org.firstinspires.ftc.teamcode.htech.config.SwerveHardwareTest;
import org.firstinspires.ftc.teamcode.htech.utils.Pose;

@Config
public class SwerveDrivetrain implements Drivetrain {
    public SwerveModuleKooky frontLeftModule, backLeftModule, backRightModule, frontRightModule;
    public SwerveModuleKooky[] modules;

    public Odometry odo;

    public static double TRACK_WIDTH = 9.92126, //distanta dintre front left si front right
            WHEEL_BASE = 9.783465; //distanta dintre front left si back left
    //distanta de la roata la centru (diagonala x+y) 6.966842 inch
    private final double R;
    public static double frontLeftOffset = 3.72, frontRightOffset = 5.5, backLeftOffset = 2.45, backRightOffset = 1.4;
    public static boolean maintainHeading = false;

    public static double TRANSLATION_SPEED = 1.0;
    public static double ROTATION_MAX_SPEED = 0.8;
    public static double DEADBAND = 0.05;

    public static double RotMinSpeed = 0.5;

    double[] ws = new double[4];
    double[] wa = new double[4];
    double max = 0.0;

    public final double minPow = 0.1;
    public static double imuOffset = 0.0;

    public static double LINEAR_ACCEL_LIMIT = 9.0;
    public static double ANGULAR_ACCEL_LIMIT = 6.0;

    public SlewRateLimiter fwLimiter, strLimiter, headingLimiter;
    private boolean locked = false;

    public SwerveDrivetrain() {
        frontLeftModule  = new SwerveModuleKooky(SwerveHardwareTest.frontLeftMotor,  SwerveHardwareTest.frontLeftServo,  new AbsoluteAnalogEncoder(SwerveHardwareTest.frontLeftEncoder,  3.3).zero(frontLeftOffset).setInverted(true));
        backLeftModule   = new SwerveModuleKooky(SwerveHardwareTest.backLeftMotor,   SwerveHardwareTest.backLeftServo,   new AbsoluteAnalogEncoder(SwerveHardwareTest.backLeftEncoder,   3.3).zero(backLeftOffset).setInverted(true));
        backRightModule  = new SwerveModuleKooky(SwerveHardwareTest.backRightMotor,  SwerveHardwareTest.backRightServo,  new AbsoluteAnalogEncoder(SwerveHardwareTest.backRightEncoder,  3.3).zero(backRightOffset).setInverted(true));
        frontRightModule = new SwerveModuleKooky(SwerveHardwareTest.frontRightMotor, SwerveHardwareTest.frontRightServo, new AbsoluteAnalogEncoder(SwerveHardwareTest.frontRightEncoder, 3.3).zero(frontRightOffset).setInverted(true));

        odo = new Odometry(SwerveHardwareTest.vs, SwerveHardwareTest.odo);
        fwLimiter = new SlewRateLimiter(LINEAR_ACCEL_LIMIT);
        strLimiter = new SlewRateLimiter(LINEAR_ACCEL_LIMIT);
        headingLimiter = new SlewRateLimiter(ANGULAR_ACCEL_LIMIT);
        modules = new SwerveModuleKooky[]{frontLeftModule, frontRightModule, backRightModule, backLeftModule};
        for (SwerveModuleKooky m : modules) m.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        R = hypot(TRACK_WIDTH, WHEEL_BASE);
    }

    public void read() {
        for (SwerveModuleKooky module : modules) module.read();
    }

    @Override
    public void set(Pose pose) {
        double x = pose.x, y = pose.y, head = pose.heading;

        double a = x - head * (WHEEL_BASE / R),
                b = x + head * (WHEEL_BASE / R),
                c = y - head * (TRACK_WIDTH / R),
                d = y + head * (TRACK_WIDTH / R);

        if (locked) {
            ws = new double[]{0, 0, 0, 0};
            wa = new double[]{-Math.PI / 4, Math.PI / 4, -Math.PI / 4, Math.PI / 4};
        } else {
            ws = new double[]{hypot(b, c), hypot(b, d), hypot(a, d), hypot(a, c)};

//            if (!maintainHeading) wa = new double[]{atan2(b, d), atan2(b, c), atan2(a, c), atan2(a, d)};
            if (x != 0 || y!= 0 || head != 0) wa = new double[]{atan2(b, d), atan2(b, c), atan2(a, c), atan2(a, d)};
        }
        max = MathUtils.max(ws);
    }

    public void write() {
        for (int i = 0; i < 4; i++) {
            SwerveModuleKooky m = modules[i];
            if (Math.abs(max) > 1) ws[i] /= max;
            m.setMotorPower(Math.abs(ws[i]));
            m.setTargetRotation(MathUtils.norm(wa[i]));
        }
    }

    public void updateModules() {
        for (SwerveModuleKooky m : modules) m.update();
    }

    public void setLocked(boolean locked){
        this.locked = locked;
    }
    public boolean isLocked(){
        return locked;
    }

    public String getTelemetry() {
        return frontLeftModule.getTelemetry("leftFrontModule") + "\n" +
                backLeftModule.getTelemetry("leftRearModule") + "\n" +
                frontRightModule.getTelemetry("rightFrontModule") + "\n" +
                backRightModule.getTelemetry("rightRearModule") + "\n";
    }


    public void updateMovementFieldCentric(Gamepad g) {
        read();
        odo.update();
        double robotHeading = odo.getHeading();

        double x = -g.left_stick_x;
        double y = g.left_stick_y;
        double w = -g.right_stick_x;

        x = deadband(x, DEADBAND);
        y = deadband(y, DEADBAND);
        w = deadband(w, DEADBAND);


        x *= TRANSLATION_SPEED;
        y *= TRANSLATION_SPEED;
        w *= ROTATION_MAX_SPEED;

        double fieldX = x* Math.cos(-robotHeading) - y * Math.sin(-robotHeading);
        double fieldY = x * Math.sin(-robotHeading) + y * Math.cos(-robotHeading);


        set(new Pose(fieldX, fieldY, w));
        write();
        updateModules();
    }
    public void updateMovement(Gamepad g) {
        read();
        double x = -g.left_stick_x;
        double y = g.left_stick_y;
        double w = -g.right_stick_x;

        x = deadband(x, DEADBAND);
        y = deadband(y, DEADBAND);
        w = deadband(w, DEADBAND);

//        double heading = -  odo.getHeading();
//
//        double cos = Math.cos(heading);
//        double sin = Math.sin(heading);
//
//        double xField = -x * cos + y * sin;
//        double yField = -x * sin - y * cos;

        x *= TRANSLATION_SPEED;
        y *= TRANSLATION_SPEED;
        w *= ROTATION_MAX_SPEED;

        x = fwLimiter.calculate(x);
        y = strLimiter.calculate(y);
        w = headingLimiter.calculate(w);


        set(new Pose(x, y, w));
        write();
        updateModules();
    }

    private double deadband(double v, double db) {
        return (Math.abs(v) < db) ? 0.0 : v;
    }
}
