package org.firstinspires.ftc.teamcode.htech.utils;

public class MotionProfile {

    private double maxVelocity;     // units / second
    private double maxAcceleration; // units / second^2

    // === STATE ===
    private double target;           // final desired position
    private double position;         // profiled position
    private double velocity;         // profiled velocity

    // === CONSTRUCTOR ===
    public MotionProfile(double maxVelocity, double maxAcceleration) {
        this.maxVelocity = maxVelocity;
        this.maxAcceleration = maxAcceleration;
    }

    // === INITIALIZATION ===
    public void reset(double currentPosition) {
        this.position = currentPosition;
        this.velocity = 0.0;
    }

    // === SET TARGET ===
    public void setTarget(double target) {
        this.target = target;
    }

    // === UPDATE LOOP ===
    public void update(double dt) {
        double error = target - position;

        // Distance required to stop
        double stoppingDistance = (velocity * velocity) / (2.0 * maxAcceleration);

        // Determine acceleration direction
        if (Math.abs(error) <= stoppingDistance) {
            // Decelerate
            velocity -= Math.signum(velocity) * maxAcceleration * dt;
        } else {
            // Accelerate toward target
            velocity += Math.signum(error) * maxAcceleration * dt;
        }

        // Clamp velocity
        velocity = clamp(velocity, -maxVelocity, maxVelocity);

        // Integrate
        position += velocity * dt;

        // Snap to target if very close
        if (Math.abs(error) < 1e-6 && Math.abs(velocity) < 1e-6) {
            position = target;
            velocity = 0.0;
        }
    }

    // === GETTERS ===
    public double getPosition() {
        return position;
    }

    public double getVelocity() {
        return velocity;
    }

    // === UTILS ===
    private double clamp(double value, double min, double max) {
        return Math.max(min, Math.min(max, value));
    }


}
