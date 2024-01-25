package frc.trigon.robot.simulation;

import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.wpilibj.simulation.ElevatorSim;
import frc.trigon.robot.constants.RobotConstants;
import frc.trigon.robot.utilities.Conversions;

public class ElevatorSimulation extends MotorSimulation {
    private final ElevatorSim elevatorSimulation;
    private final double retractedHeightMeters;
    private final double diameterMeters;

    public ElevatorSimulation(DCMotor gearbox, double gearRatio, double carriageMassKilograms, double drumRadiusMeters, double retractedHeightMeters, double maximumHeightMeters, boolean simulateGravity) {
        diameterMeters = drumRadiusMeters + drumRadiusMeters;
        this.retractedHeightMeters = retractedHeightMeters;
        elevatorSimulation = new ElevatorSim(
                gearbox,
                gearRatio,
                carriageMassKilograms,
                drumRadiusMeters,
                retractedHeightMeters,
                maximumHeightMeters,
                simulateGravity,
                retractedHeightMeters
        );
    }

    @Override
    public double getCurrent() {
        return elevatorSimulation.getCurrentDrawAmps();
    }

    @Override
    public double getPositionRevolutions() {
        return Conversions.distanceToRevolutions(elevatorSimulation.getPositionMeters() - retractedHeightMeters, diameterMeters);
    }

    @Override
    public double getVelocityRevolutionsPerSecond() {
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
