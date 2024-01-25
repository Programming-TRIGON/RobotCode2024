package frc.trigon.robot.simulation;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.simulation.SingleJointedArmSim;
import frc.trigon.robot.constants.RobotConstants;

public class SingleJointedArmSimulation extends MotorSimulation {
    private final SingleJointedArmSim armSimulation;

    public SingleJointedArmSimulation(DCMotor gearbox, double gearRatio, double armLengthMeters, double armMassKilograms, Rotation2d minimumAngle, Rotation2d maximumAngle, boolean simulateGravity) {
        armSimulation = new SingleJointedArmSim(
                gearbox,
                gearRatio,
                SingleJointedArmSim.estimateMOI(armLengthMeters, armMassKilograms),
                armLengthMeters,
                minimumAngle.getRadians(),
                maximumAngle.getRadians(),
                simulateGravity,
                minimumAngle.getRadians()
        );
    }

    @Override
    public double getCurrent() {
        return armSimulation.getCurrentDrawAmps();
    }

    @Override
    public double getPositionRevolutions() {
        return Units.radiansToRotations(armSimulation.getAngleRads());
    }

    @Override
    public double getVelocityRevolutionsPerSecond() {
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
