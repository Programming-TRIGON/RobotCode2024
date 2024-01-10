package frc.trigon.robot.motorsimulation;

import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.simulation.SingleJointedArmSim;
import frc.trigon.robot.constants.RobotConstants;

public class SingleJointedArmSimulation extends MotorSimulation {
    private final SingleJointedArmSim armSimulation;

    public SingleJointedArmSimulation(DCMotor motor, double gearRatio, double retractedArmLengthMeters, double minimumAngleRadians, double maximumAngleRadians, boolean simulateGravity, double armMass) {
        armSimulation = new SingleJointedArmSim(motor, gearRatio, SingleJointedArmSim.estimateMOI(retractedArmLengthMeters, armMass), retractedArmLengthMeters, minimumAngleRadians, maximumAngleRadians, simulateGravity, minimumAngleRadians);
    }

    @Override
    double calculateFeedforward(MotorSimulationConfiguration.FeedForwardConfigs feedForwardConfiguration, double targetPositionRadians, double targetVelocity) {
        return feedForwardConfiguration.kS * Math.signum(targetPositionRadians)
                + feedForwardConfiguration.kG * Math.cos(targetPositionRadians)
                + feedForwardConfiguration.kV * (targetVelocity)
                + feedForwardConfiguration.kA * 0;
    }

    @Override
    public double getCurrent() {
        return armSimulation.getCurrentDrawAmps();
    }

    @Override
    double getPositionRevolutions() {
        return Units.radiansToRotations(armSimulation.getAngleRads());
    }

    @Override
    double getVelocityRevolutionsPerSecond() {
        return Units.radiansToRotations(armSimulation.getVelocityRadPerSec());
    }

    @Override
    void setInputVoltage(double voltage) {
        armSimulation.setInputVoltage(voltage);
    }

    @Override
    void updateMotor() {
        armSimulation.update(RobotConstants.PERIODIC_TIME_SECONDS);
    }
}
