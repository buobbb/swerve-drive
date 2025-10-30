#include <jni.h>
#include <cmath>

// --- Constants (modifiable through setters) ---
static double hGoal = 0.985;
static double hTag = 0.75;
static double hCam = 0.0;
static double hShooter = 0.338;
static double llLateralMountError = 0.0;
static double g = 9.81;
static double speed = 0.0;
static double llMountAngle = 0.0;

// Servo + turret parameters
static double s_min = 0.0;
static double s_max = 1.0;
static double theta_min = 0.0;
static double theta_max = M_PI / 3.0; // 60 deg
static double ticks_per_turret_rev = 0.0;
static double ticks_zero = 0.0;
static int dir = 1;

// --- Utility clamp ---
double clamp(double x, double minVal, double maxVal) {
    return std::fmax(minVal, std::fmin(x, maxVal));
}

// --- JNI exposed methods ---
extern "C" {

// ---------------------
// Calculation functions
// ---------------------

JNIEXPORT jdouble JNICALL
Java_org_firstinspires_ftc_teamcode_htech_shooter_ShooterMath_updateDistance(JNIEnv*, jobject, jdouble ty) {
    return (hTag - hCam) / std::tan(llMountAngle + ty);
}

JNIEXPORT jdouble JNICALL
Java_org_firstinspires_ftc_teamcode_htech_shooter_ShooterMath_updateTurretAngle(JNIEnv*, jobject, jdouble tx, jdouble distance) {
    return -tx + std::atan2(llLateralMountError, distance);
}

JNIEXPORT jdouble JNICALL
Java_org_firstinspires_ftc_teamcode_htech_shooter_ShooterMath_updatePitchAngle(JNIEnv*, jobject, jdouble distance) {
    double underRoot = speed * speed * speed * speed - g * (g * distance * distance + 2 * (hGoal - hShooter) * speed * speed);
    if (underRoot < 0) return NAN;
    double pitchAngle = std::atan((speed * speed - std::sqrt(underRoot)) / (g * distance));
    return pitchAngle;
}

JNIEXPORT jdouble JNICALL
Java_org_firstinspires_ftc_teamcode_htech_shooter_ShooterMath_angleToServoPos(JNIEnv*, jobject, jdouble radians) {
    radians = clamp(radians, theta_min, theta_max);
    double s_target = s_min + (radians - theta_min) * (s_max - s_min) / (theta_max - theta_min);
    return clamp(s_target, 0.0, 1.0);
}

JNIEXPORT jint JNICALL
Java_org_firstinspires_ftc_teamcode_htech_shooter_ShooterMath_angleToTicks(JNIEnv*, jobject, jdouble radians) {
    return (int)(ticks_zero + dir * (radians / (2 * M_PI)) * ticks_per_turret_rev);
}

JNIEXPORT jdouble JNICALL
Java_org_firstinspires_ftc_teamcode_htech_shooter_ShooterMath_wrapRadians(JNIEnv*, jobject, jdouble angle) {
    angle = std::fmod(angle + M_PI, 2 * M_PI);
    if (angle < 0)
        angle += 2 * M_PI;
    return angle - M_PI;
}

// ---------------------
// Setter functions
// ---------------------

JNIEXPORT void JNICALL
Java_org_firstinspires_ftc_teamcode_htech_shooter_ShooterMath_setSpeed(JNIEnv*, jobject, jdouble s) {
speed = s;
}

JNIEXPORT void JNICALL
Java_org_firstinspires_ftc_teamcode_htech_shooter_ShooterMath_setLLMountAngle(JNIEnv*, jobject, jdouble angle) {
llMountAngle = angle;
}

JNIEXPORT void JNICALL
Java_org_firstinspires_ftc_teamcode_htech_shooter_ShooterMath_setHGoal(JNIEnv*, jobject, jdouble val) {
hGoal = val;
}

JNIEXPORT void JNICALL
Java_org_firstinspires_ftc_teamcode_htech_shooter_ShooterMath_setHShooter(JNIEnv*, jobject, jdouble val) {
hShooter = val;
}

JNIEXPORT void JNICALL
Java_org_firstinspires_ftc_teamcode_htech_shooter_ShooterMath_setHTag(JNIEnv*, jobject, jdouble val) {
hTag = val;
}

JNIEXPORT void JNICALL
Java_org_firstinspires_ftc_teamcode_htech_shooter_ShooterMath_setHCam(JNIEnv*, jobject, jdouble val) {
hCam = val;
}

JNIEXPORT void JNICALL
Java_org_firstinspires_ftc_teamcode_htech_shooter_ShooterMath_setLLLateralMountError(JNIEnv*, jobject, jdouble val) {
llLateralMountError = val;
}

JNIEXPORT void JNICALL
Java_org_firstinspires_ftc_teamcode_htech_shooter_ShooterMath_setServoParams(JNIEnv*, jobject,
jdouble sMin, jdouble sMax,
jdouble tMin, jdouble tMax) {
s_min = sMin;
s_max = sMax;
theta_min = tMin;
theta_max = tMax;
}

JNIEXPORT void JNICALL
Java_org_firstinspires_ftc_teamcode_htech_shooter_ShooterMath_setTurretParams(JNIEnv*, jobject,
jdouble ticksPerRev, jdouble ticksZero,
jint direction) {
ticks_per_turret_rev = ticksPerRev;
ticks_zero = ticksZero;
dir = direction;
}

} // extern "C"
