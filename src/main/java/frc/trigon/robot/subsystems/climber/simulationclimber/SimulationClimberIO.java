package frc.trigon.robot.subsystems.climber.simulationclimber;

import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import frc.trigon.robot.simulation.ElevatorSimulation;
import frc.trigon.robot.subsystems.climber.ClimberConstants;
import frc.trigon.robot.subsystems.climber.ClimberIO;
import frc.trigon.robot.subsystems.climber.ClimberInputsAutoLogged;
import frc.trigon.robot.utilities.Conversions;

public class SimulationClimberIO extends ClimberIO {
    private final ElevatorSimulation masterMotor = SimulationClimberConstants.MOTOR;
    private final MotionMagicVoltage positionRequest = new MotionMagicVoltage(0);
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
    protected void setTargetPositionMeters(double targetPositionMeters) {
        masterMotor.setControl(positionRequest.withPosition(Conversions.distanceToRevolutions(targetPositionMeters, ClimberConstants.DIAMETER_METERS)));
    }

    @Override
    protected void setTargetVoltage(double targetVoltage) {
        masterMotor.setControl(voltageRequest.withOutput(targetVoltage));
    }

    @Override
    protected void stop() {
        masterMotor.stop();
    }
}
