package frc.trigon.robot.subsystems.elevator.simulationelevator;

import com.ctre.phoenix6.controls.MotionMagicVoltage;
import frc.trigon.robot.simulation.ElevatorSimulation;
import frc.trigon.robot.subsystems.elevator.ElevatorIO;
import frc.trigon.robot.subsystems.elevator.ElevatorInputsAutoLogged;

public class SimulationElevatorIO extends ElevatorIO {
    private static final ElevatorSimulation motor = SimulationElevatorConstants.MOTOR;
    private static final MotionMagicVoltage positionRequest = new MotionMagicVoltage(0).withEnableFOC(SimulationElevatorConstants.FOC_ENABLED);

    @Override
    protected void updateInputs(ElevatorInputsAutoLogged inputs) {
        inputs.motorVoltage = motor.getVoltage();
        inputs.motorPositionMeters = motor.getPosition();
        inputs.motorVelocityMetersPerSecond = motor.getVelocity();
    }

    @Override
    protected void setTargetPosition(double targetPositionMeters) {
        motor.setControl(positionRequest.withPosition(targetPositionMeters));
    }

    @Override
    protected void stopMotors() {
        motor.stop();
    }
}
