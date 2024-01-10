package frc.trigon.robot.motorsimulation;

import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;
import frc.trigon.robot.constants.RobotConstants;

public class SimpleMotorSimulation extends MotorSimulation {
    private final DCMotorSim motorSimulation;

    public SimpleMotorSimulation(DCMotor motor, double gearRatio, double momentOfInertia) {
        motorSimulation = new DCMotorSim(motor, gearRatio, momentOfInertia);
    }

    @Override
    double calculateFeedforward(MotorSimulationConfiguration.FeedForwardConfigs feedForwardConfiguration, double targetPositionRadians, double targetVelocity) {
        return feedForwardConfiguration.kS * Math.signum(targetVelocity)
                + feedForwardConfiguration.kV * targetVelocity
                + feedForwardConfiguration.kA * 0;
    }

    @Override
    public double getCurrent() {
        return motorSimulation.getCurrentDrawAmps();
    }

    @Override
    double getPositionRevolutions() {
        return Units.radiansToRotations(motorSimulation.getAngularPositionRad());
    }

    @Override
    double getVelocityRevolutionsPerSecond() {
        return Units.radiansToRotations(motorSimulation.getAngularVelocityRadPerSec());
    }

    @Override
    void setInputVoltage(double voltage) {
        motorSimulation.setInputVoltage(voltage);
    }

    @Override
    void updateMotor() {
        motorSimulation.update(RobotConstants.PERIODIC_TIME_SECONDS);
    }
}
