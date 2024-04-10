package frc.trigon.robot.simulation;

import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.simulation.FlywheelSim;
import frc.trigon.robot.constants.RobotConstants;

public class FlywheelSimulation extends MotorSimulation {
    private final FlywheelSim flywheelSimulation;
    private double lastPositionRadians = 0;

    public FlywheelSimulation(DCMotor gearbox, double gearRatio, double momentOfInertia) {
        flywheelSimulation = new FlywheelSim(gearbox, gearRatio, momentOfInertia);
    }

    public FlywheelSimulation(DCMotor gearbox, double gearRatio, double kv, double ka) {
        flywheelSimulation = new FlywheelSim(LinearSystemId.identifyVelocitySystem(kv, ka), gearbox, gearRatio);
    }

    @Override
    public double getCurrent() {
        return flywheelSimulation.getCurrentDrawAmps();
    }

    @Override
    public double getPositionRevolutions() {
        return Units.radiansToRotations(lastPositionRadians);
    }

    @Override
    public double getVelocityRevolutionsPerSecond() {
        return Units.radiansToRotations(flywheelSimulation.getAngularVelocityRadPerSec());
    }

    @Override
    void setInputVoltage(double voltage) {
        flywheelSimulation.setInputVoltage(voltage);
    }

    @Override
    void updateMotor() {
        flywheelSimulation.update(RobotConstants.PERIODIC_TIME_SECONDS);
        lastPositionRadians = lastPositionRadians + flywheelSimulation.getAngularVelocityRadPerSec() * RobotConstants.PERIODIC_TIME_SECONDS;
    }
}
