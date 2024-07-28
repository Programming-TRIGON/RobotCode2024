package frc.trigon.robot.hardware.simulation;

/**
 * An abstract class to simulate the physics of a motor.
 */
public abstract class MotorPhysicsSimulation {
    public abstract double getCurrent();

    public abstract double getPositionRevolutions();

    public abstract double getVelocityRevolutionsPerSecond();

    public abstract void setInputVoltage(double voltage);

    public abstract void updateMotor();
}
