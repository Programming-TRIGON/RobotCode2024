package frc.trigon.robot.motorsimulation;

import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.wpilibj.simulation.ElevatorSim;
import frc.trigon.robot.constants.RobotConstants;
import frc.trigon.robot.utilities.Conversions;

public class ElevatorSimulation extends MotorSimulation {
    private final ElevatorSim elevatorSimulation;
    private final double diameterMeters;

    public ElevatorSimulation(DCMotor motor, double gearRatio, double mass, double drumRadiusMeters, double retractedArmLengthMeters, double maximumAngleRadians, boolean simulateGravity) {
        this.diameterMeters = drumRadiusMeters + drumRadiusMeters;
        elevatorSimulation = new ElevatorSim(motor, gearRatio, mass, drumRadiusMeters, retractedArmLengthMeters, maximumAngleRadians, simulateGravity, retractedArmLengthMeters);
    }

    @Override
    public double getCurrent() {
        return elevatorSimulation.getCurrentDrawAmps();
    }

    @Override
    double calculateFeedforward(MotorSimulationConfiguration.FeedForwardConfigs feedForwardConfiguration, double targetPositionRadians, double targetVelocity) {
        return feedForwardConfiguration.kS * Math.signum(targetVelocity)
                + feedForwardConfiguration.kG
                + feedForwardConfiguration.kV * targetVelocity
                + feedForwardConfiguration.kA * 0;
    }

    @Override
    double getPositionRevolutions() {
        return Conversions.distanceToRevolutions(elevatorSimulation.getPositionMeters(), diameterMeters);
    }

    @Override
    double getVelocityRevolutionsPerSecond() {
        return Conversions.distanceToRevolutions(elevatorSimulation.getVelocityMetersPerSecond(), diameterMeters);
    }

    @Override
    void setInputVoltage(double voltage) {
        elevatorSimulation.setInputVoltage(voltage);
    }

    @Override
    void updateMotor() {
        elevatorSimulation.update(RobotConstants.PERIODIC_TIME_SECONDS);
    }
}
