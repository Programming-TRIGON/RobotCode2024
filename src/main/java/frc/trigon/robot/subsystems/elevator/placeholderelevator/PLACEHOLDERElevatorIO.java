package frc.trigon.robot.subsystems.elevator.placeholderelevator;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;
import frc.trigon.robot.subsystems.elevator.ElevatorConstants;
import frc.trigon.robot.subsystems.elevator.ElevatorIO;
import frc.trigon.robot.subsystems.elevator.ElevatorInputsAutoLogged;
import frc.trigon.robot.utilities.Conversions;

public class PLACEHOLDERElevatorIO extends ElevatorIO {
    private final TalonFX
            masterMotor = PLACEHOLDERElevatorConstants.MASTER_MOTOR,
            followerMotor = PLACEHOLDERElevatorConstants.FOLLOWER_MOTOR;
    private final MotionMagicVoltage positionRequest = new MotionMagicVoltage(0).withEnableFOC(PLACEHOLDERElevatorConstants.FOC_ENABLED);

    @Override
    protected void updateInputs(ElevatorInputsAutoLogged inputs) {
        refreshStatusSignals();
        inputs.motorVoltage = PLACEHOLDERElevatorConstants.MOTOR_VOLTAGE_STATUS_SIGNAL.getValue();
        inputs.positionMeters = getEncoderPositionMeters();
        inputs.velocityMetersPerSecond = getEncoderVelocityMetersPerSecond();
        inputs.profiledSetpoint = PLACEHOLDERElevatorConstants.MOTOR_SETPOINT_STATUS_SIGNAL.getValue();
    }

    @Override
    protected void setTargetPosition(double targetPositionMeters) {
        masterMotor.setControl(positionRequest.withPosition(Conversions.distanceToRevolutions(targetPositionMeters, getWheelDiameter())));
    }

    @Override
    protected void stopMotors() {
        masterMotor.stopMotor();
    }

    @Override
    protected void setBrake(boolean brake) {
        masterMotor.setNeutralMode(brake ? NeutralModeValue.Brake : NeutralModeValue.Coast);
        followerMotor.setNeutralMode(brake ? NeutralModeValue.Brake : NeutralModeValue.Coast);
    }

    private double getEncoderPositionMeters() {
        return Conversions.revolutionsToDistance(PLACEHOLDERElevatorConstants.ENCODER_POSITION_STATUS_SIGNAL.getValue(), getWheelDiameter());
    }

    private double getEncoderVelocityMetersPerSecond() {
        return Conversions.revolutionsToDistance(PLACEHOLDERElevatorConstants.ENCODER_VELOCITY_STATUS_SIGNAL.getValue(), getWheelDiameter());
    }

    private void refreshStatusSignals() {
        BaseStatusSignal.refreshAll(
                PLACEHOLDERElevatorConstants.ENCODER_POSITION_STATUS_SIGNAL,
                PLACEHOLDERElevatorConstants.ENCODER_VELOCITY_STATUS_SIGNAL,
                PLACEHOLDERElevatorConstants.MOTOR_VOLTAGE_STATUS_SIGNAL,
                PLACEHOLDERElevatorConstants.MOTOR_SETPOINT_STATUS_SIGNAL
        );
    }

    private double getWheelDiameter() {
        return ElevatorConstants.DRUM_RADIUS_METERS * 2;
    }
}
