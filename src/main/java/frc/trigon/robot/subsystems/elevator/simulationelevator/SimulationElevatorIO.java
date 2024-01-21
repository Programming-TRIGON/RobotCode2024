package frc.trigon.robot.subsystems.elevator.simulationelevator;

import com.ctre.phoenix6.controls.MotionMagicVoltage;
import frc.trigon.robot.simulation.ElevatorSimulation;
import frc.trigon.robot.subsystems.elevator.ElevatorConstants;
import frc.trigon.robot.subsystems.elevator.ElevatorIO;
import frc.trigon.robot.subsystems.elevator.ElevatorInputsAutoLogged;
import frc.trigon.robot.utilities.Conversions;

public class SimulationElevatorIO extends ElevatorIO {
    private final ElevatorSimulation motor = SimulationElevatorConstants.MOTOR;
    private final MotionMagicVoltage positionRequest = new MotionMagicVoltage(0);

    @Override
    protected void updateInputs(ElevatorInputsAutoLogged inputs) {
        inputs.motorVoltage = motor.getVoltage();
        inputs.motorPositionMeters = getMotorPositionMeters();
        inputs.motorVelocityMetersPerSecond = getMotorVelocityMetersPerSecond();
    }

    @Override
    protected void setTargetPosition(double targetPositionMeters) {
        motor.setControl(positionRequest.withPosition(targetPositionMeters));
    }

    @Override
    protected void stopMotors() {
        motor.stop();
    }

    private double getMotorPositionMeters() {
        return Conversions.revolutionsToDistance(motor.getPosition(), ElevatorConstants.DRUM_RADIUS_METERS);
    }

    private double getMotorVelocityMetersPerSecond() {
        return Conversions.revolutionsToDistance(motor.getVelocity(), ElevatorConstants.DRUM_RADIUS_METERS);
    }
}