package frc.trigon.robot.subsystems.climber.simulationclimber;

import com.ctre.phoenix6.controls.DynamicMotionMagicVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import frc.trigon.robot.simulation.ElevatorSimulation;
import frc.trigon.robot.subsystems.climber.ClimberConstants;
import frc.trigon.robot.subsystems.climber.ClimberIO;
import frc.trigon.robot.subsystems.climber.ClimberInputsAutoLogged;
import frc.trigon.robot.utilities.Conversions;

public class SimulationClimberIO extends ClimberIO {
    private final ElevatorSimulation masterMotor = SimulationClimberConstants.MOTOR;
    private final DynamicMotionMagicVoltage
            nonClimbingPositionRequest = new DynamicMotionMagicVoltage(0, SimulationClimberConstants.MAX_NON_CLIMBING_VELOCITY, SimulationClimberConstants.MAX_NON_CLIMBING_ACCELERATION, 0).withSlot(SimulationClimberConstants.NON_CLIMBING_SLOT),
            climbingPositionRequest = new DynamicMotionMagicVoltage(0, SimulationClimberConstants.MAX_CLIMBING_VELOCITY, SimulationClimberConstants.MAX_CLIMBING_ACCELERATION, 0).withSlot(SimulationClimberConstants.CLIMBING_SLOT);
    private final VoltageOut voltageRequest = new VoltageOut(0);

    @Override
    protected void updateInputs(ClimberInputsAutoLogged inputs) {
        inputs.encoderPositionMeters = Conversions.revolutionsToDistance(masterMotor.getPositionRevolutions(), ClimberConstants.DIAMETER_METERS);
        inputs.encoderVelocityMetersPerSecond = Conversions.revolutionsToDistance(masterMotor.getVelocityRevolutionsPerSecond(), ClimberConstants.DIAMETER_METERS);
        inputs.motorProfiledSetpointMeters = Conversions.revolutionsToDistance(masterMotor.getProfiledSetpointRevolutions(), ClimberConstants.DIAMETER_METERS);
        inputs.motorVoltage = masterMotor.getVoltage();
        inputs.motorCurrent = masterMotor.getCurrent();
    }

    @Override
    protected void setTargetPositionMeters(double targetPositionMeters, boolean affectedByWeight) {
        final double targetPositionRevolutions = Conversions.distanceToRevolutions(targetPositionMeters, ClimberConstants.DIAMETER_METERS);
        masterMotor.setControl(determineRequest(affectedByWeight).withPosition(targetPositionRevolutions));
    }

    @Override
    protected void setTargetVoltage(double targetVoltage) {
        masterMotor.setControl(voltageRequest.withOutput(targetVoltage));
    }

    @Override
    protected void stop() {
        masterMotor.stop();
    }

    private DynamicMotionMagicVoltage determineRequest(boolean affectedByWeight) {
        return affectedByWeight ? climbingPositionRequest : nonClimbingPositionRequest;
    }
}
