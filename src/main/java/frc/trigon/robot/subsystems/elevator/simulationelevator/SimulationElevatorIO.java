package frc.trigon.robot.subsystems.elevator.simulationelevator;

import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import frc.trigon.robot.simulation.ElevatorSimulation;
import frc.trigon.robot.subsystems.elevator.ElevatorIO;
import frc.trigon.robot.subsystems.elevator.ElevatorInputsAutoLogged;

public class SimulationElevatorIO extends ElevatorIO {
    private final ElevatorSimulation motor = SimulationElevatorConstants.MOTOR;
    private final MotionMagicVoltage positionRequest = new MotionMagicVoltage(0);
    private final VoltageOut voltageRequest = new VoltageOut(0);

    @Override
    protected void updateInputs(ElevatorInputsAutoLogged inputs) {
        inputs.motorVoltage = motor.getVoltage();
        inputs.positionRevolutions = motor.getPositionRevolutions();
        inputs.velocityRevolutionsPerSecond = motor.getVelocityRevolutionsPerSecond();
        inputs.profiledSetpointRevolutions = motor.getProfiledSetpointRevolutions();
    }

    @Override
    protected void setTargetPosition(double targetPositionRevolutions, double speedPercentage) {
        // TODO: SPeed percentage
        motor.setControl(positionRequest.withPosition(targetPositionRevolutions));
    }

    @Override
    protected void setTargetVoltage(double voltage) {
        motor.setControl(voltageRequest.withOutput(voltage));
    }

    @Override
    protected void stop() {
        motor.stop();
    }
}
