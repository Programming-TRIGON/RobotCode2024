package frc.trigon.robot.subsystems.elevator.simulationelevator;

import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import frc.trigon.robot.simulation.ElevatorSimulation;
import frc.trigon.robot.subsystems.elevator.ElevatorConstants;
import frc.trigon.robot.subsystems.elevator.ElevatorIO;
import frc.trigon.robot.subsystems.elevator.ElevatorInputsAutoLogged;
import frc.trigon.robot.utilities.Conversions;

public class SimulationElevatorIO extends ElevatorIO {
    private final ElevatorSimulation motor = SimulationElevatorConstants.MOTOR;
    private final MotionMagicVoltage positionRequest = new MotionMagicVoltage(0);
    private final VoltageOut voltageRequest = new VoltageOut(0);

    @Override
    protected void updateInputs(ElevatorInputsAutoLogged inputs) {
        inputs.motorVoltage = motor.getVoltage();
        inputs.positionMeters = getMotorPositionMeters();
        inputs.velocityMetersPerSecond = getMotorVelocityMetersPerSecond();
        inputs.profiledSetpointMeters = getProfiledSetpointMeters();
    }

    @Override
    protected void setTargetPosition(double targetPositionMeters) {
        motor.setControl(positionRequest.withPosition(Conversions.distanceToRevolutions(targetPositionMeters, ElevatorConstants.DRUM_DIAMETER_METERS)));
    }

    @Override
    protected void setTargetVoltage(double voltage) {
        motor.setControl(voltageRequest.withOutput(voltage));
    }

    @Override
    protected void stop() {
        motor.stop();
    }

    private double getMotorPositionMeters() {
        return Conversions.revolutionsToDistance(motor.getPositionRevolutions(), ElevatorConstants.DRUM_DIAMETER_METERS);
    }

    private double getMotorVelocityMetersPerSecond() {
        return Conversions.revolutionsToDistance(motor.getVelocityRevolutionsPerSecond(), ElevatorConstants.DRUM_DIAMETER_METERS);
    }

    private double getProfiledSetpointMeters() {
        return Conversions.revolutionsToDistance(motor.getProfiledSetpointRevolutions(), ElevatorConstants.DRUM_DIAMETER_METERS);
    }
}
