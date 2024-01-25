package frc.trigon.robot.subsystems.elevator.simulationelevator;

import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import edu.wpi.first.units.Measure;
import edu.wpi.first.units.Units;
import edu.wpi.first.units.Voltage;
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
        inputs.profiledSetpointMeters = Conversions.revolutionsToDistance(motor.getProfiledSetpoint(), ElevatorConstants.DRUM_DIAMETER_METERS);
    }

    @Override
    protected void setTargetPosition(double targetPositionMeters) {
        motor.setControl(positionRequest.withPosition(Conversions.distanceToRevolutions(targetPositionMeters, ElevatorConstants.DRUM_DIAMETER_METERS)));
    }

    @Override
    protected void setMotorVoltage(Measure<Voltage> voltageMeasure) {
        motor.setControl(voltageRequest.withOutput(voltageMeasure.in(Units.Volts)));
    }

    @Override
    protected void stop() {
        motor.stop();
    }

    private double getMotorPositionMeters() {
        return Conversions.revolutionsToDistance(motor.getPosition(), ElevatorConstants.DRUM_DIAMETER_METERS);
    }

    private double getMotorVelocityMetersPerSecond() {
        return Conversions.revolutionsToDistance(motor.getVelocity(), ElevatorConstants.DRUM_DIAMETER_METERS);
    }
}
