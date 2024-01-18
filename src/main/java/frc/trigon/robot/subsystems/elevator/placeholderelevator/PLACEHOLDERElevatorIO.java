package frc.trigon.robot.subsystems.elevator.placeholderelevator;

import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;
import frc.trigon.robot.subsystems.elevator.ElevatorIO;
import frc.trigon.robot.subsystems.elevator.ElevatorInputsAutoLogged;
import frc.trigon.robot.subsystems.elevator.simulationelevator.SimulationElevatorConstants;
import frc.trigon.robot.utilities.Conversions;

public class PLACEHOLDERElevatorIO extends ElevatorIO {
    private final TalonFX
            masterMotor = PLACEHOLDERElevatorConstants.MASTER_MOTOR,
            followerMotor = PLACEHOLDERElevatorConstants.FOLLOWER_MOTOR;
    private final MotionMagicVoltage positionRequest = new MotionMagicVoltage(0).withEnableFOC(PLACEHOLDERElevatorConstants.FOC_ENABLED);

    @Override
    protected void updateInputs(ElevatorInputsAutoLogged inputs) {
        inputs.motorVoltage = PLACEHOLDERElevatorConstants.MASTER_MOTOR_VOLTAGE_STATUS_SIGNAL.refresh().getValue();
        inputs.motorPositionMeters = getEncoderPositionMeters();
        inputs.motorVelocityMetersPerSecond = getEncoderVelocityMetersPerSecond();
    }

    @Override
    protected void setTargetPosition(double targetPositionMeters) {
        masterMotor.setControl(positionRequest.withPosition(targetPositionMeters));
    }

    @Override
    protected void stopMotors() {
        masterMotor.stopMotor();
        followerMotor.stopMotor();
    }

    @Override
    protected void setBrake(boolean brake) {
        masterMotor.setNeutralMode(brake ? NeutralModeValue.Brake : NeutralModeValue.Coast);
        followerMotor.setNeutralMode(brake ? NeutralModeValue.Brake : NeutralModeValue.Coast);
    }

    private double getEncoderPositionMeters() {
        return PLACEHOLDERElevatorConstants.ENCODER_POSITION_STATUS_SIGNAL.refresh().getValue();
    }

    private double getEncoderVelocityMetersPerSecond() {
        return Conversions.revolutionsToDistance(PLACEHOLDERElevatorConstants.ENCODER_VELOCITY_STATUS_SIGNAL.refresh().getValue(), SimulationElevatorConstants.DRUM_RADIUS_METERS);
    }
}
