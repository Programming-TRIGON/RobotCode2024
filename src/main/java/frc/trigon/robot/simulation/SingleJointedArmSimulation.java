package frc.trigon.robot.simulation;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.simulation.SingleJointedArmSim;
import frc.trigon.robot.constants.RobotConstants;

public class SingleJointedArmSimulation extends MotorSimulation {
    private final SingleJointedArmSim armSimulation;

    public SingleJointedArmSimulation(DCMotor gearbox, double gearRatio, double armLengthMeters, double armMass, Rotation2d minimumAngle, Rotation2d maximumAngle, boolean simulateGravity) {
        armSimulation = new SingleJointedArmSim(
                gearbox,
                gearRatio,
                SingleJointedArmSim.estimateMOI(armLengthMeters, armMass),
                armLengthMeters,
                minimumAngle.getRadians(),
                maximumAngle.getRadians(),
                simulateGravity,
                minimumAngle.getRadians()
        );
    }

    @Override
    double calculateFeedforward(MotorSimulationConfiguration.FeedforwardConfigs feedforwardConfiguration, double targetPositionRadians, double targetVelocity) {
        return feedforwardConfiguration.kS * Math.signum(targetPositionRadians)
                + feedforwardConfiguration.kG * Math.cos(targetPositionRadians)
                + feedforwardConfiguration.kV * (targetVelocity)
                + feedforwardConfiguration.kA * 0;
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
