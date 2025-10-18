package org.firstinspires.ftc.teamcode.htech.swerve;

import static java.lang.Math.atan2;
import static java.lang.Math.hypot;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Gamepad;

import org.firstinspires.ftc.teamcode.htech.config.SwerveHardware;
import org.firstinspires.ftc.teamcode.htech.utils.Pose;

@Config
public class SwerveDrivetrain implements Drivetrain {
    public SwerveModule frontLeftModule, backLeftModule, backRightModule, frontRightModule;
    public SwerveModule[] modules;

    public static double TRACK_WIDTH = 9, //distanta dintre front left si front right
            WHEEL_BASE = 9; //distanta dintre front left si back left
    private final double R;
    public static double frontLeftOffset = 2, frontRightOffset = 0, backLeftOffset = 0, backRightOffset = -0.055;
    public static boolean maintainHeading = false;

    public static double TRANSLATION_SPEED = 1.0;
    public static double ROTATION_SPEED = 0.75;
    public static double DEADBAND = 0.05;

    double[] ws = new double[4];
    double[] wa = new double[4];
    double max = 0.0;

    public final double minPow = 0.1;
    public static double imuOffset = 0.0;

    private boolean locked = false;

    public SwerveDrivetrain() {
        frontLeftModule  = new SwerveModule(SwerveHardware.frontLeftMotor,  SwerveHardware.frontLeftServo,  new AbsoluteAnalogEncoder(SwerveHardware.frontLeftEncoder,  3.3).zero(frontLeftOffset).setInverted(true));
        backLeftModule   = new SwerveModule(SwerveHardware.backLeftMotor,   SwerveHardware.backLeftServo,   new AbsoluteAnalogEncoder(SwerveHardware.backLeftEncoder,   3.3).zero(backLeftOffset).setInverted(true));
        backRightModule  = new SwerveModule(SwerveHardware.backRightMotor,  SwerveHardware.backRightServo,  new AbsoluteAnalogEncoder(SwerveHardware.backRightEncoder,  3.3).zero(backRightOffset).setInverted(true));
        frontRightModule = new SwerveModule(SwerveHardware.frontRightMotor, SwerveHardware.frontRightServo, new AbsoluteAnalogEncoder(SwerveHardware.frontRightEncoder, 3.3).zero(frontRightOffset).setInverted(true));

        modules = new SwerveModule[]{frontLeftModule, frontRightModule, backRightModule, backLeftModule};
        for (SwerveModule m : modules) m.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        R = hypot(TRACK_WIDTH, WHEEL_BASE);
    }

    public void read() {
        for (SwerveModule module : modules) module.read();
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
            wa = new double[]{Math.PI / 4, -Math.PI / 4, Math.PI / 4, -Math.PI / 4};
        } else {
            ws = new double[]{hypot(b, c), hypot(b, d), hypot(a, d), hypot(a, c)};
            if (!maintainHeading) wa = new double[]{atan2(b, c), atan2(b, d), atan2(a, d), atan2(a, c)};
        }
        max = MathUtils.max(ws);
    }

    public void write() {
        for (int i = 0; i < 4; i++) {
            SwerveModule m = modules[i];
            if (Math.abs(max) > 1) ws[i] /= max;
            m.setMotorPower(Math.abs(ws[i]));
            m.setTargetRotation(MathUtils.norm(wa[i]));
        }
    }

    public void updateModules() {
        for (SwerveModule m : modules) m.update();
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

    public void updateMovement(Gamepad g) {
        read();
        double x = g.left_stick_x;
        double y = -g.left_stick_y;
        double w = g.right_stick_x * ROTATION_SPEED;

        x = deadband(x, DEADBAND);
        y = deadband(y, DEADBAND);
        w = deadband(w, DEADBAND);

        x *= TRANSLATION_SPEED;
        y *= TRANSLATION_SPEED;
        w *= TRANSLATION_SPEED;


        set(new Pose(x, y, w));
        write();
        updateModules();
    }

    private double deadband(double v, double db) {
        return (Math.abs(v) < db) ? 0.0 : v;
    }
}
