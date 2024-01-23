package frc.trigon.robot.subsystems.climber.simulationclimber;

import com.ctre.phoenix6.controls.MotionMagicVoltage;
import frc.trigon.robot.simulation.ElevatorSimulation;
import frc.trigon.robot.subsystems.climber.ClimberConstants;
import frc.trigon.robot.subsystems.climber.ClimberIO;
import frc.trigon.robot.subsystems.climber.ClimberInputsAutoLogged;
import frc.trigon.robot.utilities.Conversions;

public class SimulationClimberIO extends ClimberIO {
    private final ElevatorSimulation
            masterMotor = SimulationClimberConstants.MASTER_MOTOR,
            followerMotor = SimulationClimberConstants.FOLLOWER_MOTOR;
    private final MotionMagicVoltage
            masterMotorPositionRequest = new MotionMagicVoltage(0),
            followerMotorPositionRequest = new MotionMagicVoltage(0);

    @Override
    protected void updateInputs(ClimberInputsAutoLogged inputs) {
        inputs.encoderPositionMeters = Conversions.revolutionsToDistance(masterMotor.getPosition(), ClimberConstants.DIAMETER_METERS);
        inputs.encoderVelocityMetersPerSecond = Conversions.revolutionsToDistance(masterMotor.getVelocity(), ClimberConstants.DIAMETER_METERS);
        inputs.motorProfiledSetpointMeters = Conversions.revolutionsToDistance(masterMotor.getProfiledSetpoint(), ClimberConstants.DIAMETER_METERS);
        inputs.motorVoltage = masterMotor.getVoltage();
        inputs.motorCurrent = masterMotor.getCurrent();
    }

    @Override
    protected void setPositionMeters(double targetPositionMeters) {
        masterMotor.setControl(masterMotorPositionRequest.withPosition(Conversions.distanceToRevolutions(targetPositionMeters, ClimberConstants.DIAMETER_METERS)));
        followerMotor.setControl(followerMotorPositionRequest.withPosition(Conversions.distanceToRevolutions(targetPositionMeters, ClimberConstants.DIAMETER_METERS)));
    }

    @Override
    protected void stop() {
        masterMotor.stop();
        followerMotor.stop();
    }
}
