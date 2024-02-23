package frc.trigon.robot.subsystems.climber.simulationclimber;

import com.ctre.phoenix6.controls.DynamicMotionMagicVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import frc.trigon.robot.simulation.ElevatorSimulation;
import frc.trigon.robot.subsystems.climber.ClimberIO;
import frc.trigon.robot.subsystems.climber.ClimberInputsAutoLogged;

public class SimulationClimberIO extends ClimberIO {
    private final ElevatorSimulation masterMotor = SimulationClimberConstants.MOTOR;
    private final DynamicMotionMagicVoltage
            nonClimbingPositionRequest = new DynamicMotionMagicVoltage(0, SimulationClimberConstants.MAX_NON_CLIMBING_VELOCITY, SimulationClimberConstants.MAX_NON_CLIMBING_ACCELERATION, 0).withSlot(SimulationClimberConstants.NON_CLIMBING_SLOT),
            climbingPositionRequest = new DynamicMotionMagicVoltage(0, SimulationClimberConstants.MAX_CLIMBING_VELOCITY, SimulationClimberConstants.MAX_CLIMBING_ACCELERATION, 0).withSlot(SimulationClimberConstants.CLIMBING_SLOT);
    private final VoltageOut voltageRequest = new VoltageOut(0);

    @Override
    protected void updateInputs(ClimberInputsAutoLogged inputs) {
        inputs.positionRevolutions = masterMotor.getPositionRevolutions();
        inputs.velocityRevolutionsPerSecond = masterMotor.getVelocityRevolutionsPerSecond();
        inputs.profiledSetpointRevolutions = masterMotor.getProfiledSetpointRevolutions();
        inputs.motorVoltage = masterMotor.getVoltage();
        inputs.motorCurrent = masterMotor.getCurrent();
    }

    @Override
    protected void setTargetPosition(double targetPositionRevolutions, boolean affectedByWeight) {
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
