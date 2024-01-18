package frc.trigon.robot.subsystems.elevator.simulationelevator;

import com.ctre.phoenix6.controls.MotionMagicVoltage;
import frc.trigon.robot.simulation.ElevatorSimulation;
import frc.trigon.robot.subsystems.elevator.ElevatorIO;
import frc.trigon.robot.subsystems.elevator.ElevatorInputsAutoLogged;

public class SimulationElevatorIO extends ElevatorIO {
    private static final ElevatorSimulation motor = SimulationElevatorConstants.MOTOR;
    private static final MotionMagicVoltage voltageRequest = new MotionMagicVoltage(0).withEnableFOC(SimulationElevatorConstants.FOC_ENABLED);

    @Override
    protected void updateInputs(ElevatorInputsAutoLogged inputs) {
        inputs.masterMotorVoltage = motor.getVoltage();
        inputs.followerMotorVoltage = motor.getVoltage();

        inputs.masterMotorPositionMeters = motor.getPosition();
        inputs.followerMotorPositionMeters = motor.getPosition();

        inputs.masterMotorVelocityMetersPerSecond = motor.getVelocity();
        inputs.followerMotorVelocityMetersPerSecond = motor.getVelocity();
    }

    @Override
    protected void setTargetState(double targetStateMeters) {
        motor.setControl(voltageRequest.withPosition(targetStateMeters));
    }

    @Override
    protected void stopMotors() {
        motor.stop();
    }
}
