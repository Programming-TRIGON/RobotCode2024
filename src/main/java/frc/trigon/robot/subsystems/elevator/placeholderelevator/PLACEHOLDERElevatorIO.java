package frc.trigon.robot.subsystems.elevator.placeholderelevator;

import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import frc.trigon.robot.subsystems.elevator.ElevatorIO;
import frc.trigon.robot.subsystems.elevator.ElevatorInputsAutoLogged;

public class PLACEHOLDERElevatorIO extends ElevatorIO {
    private final TalonFX
            masterMotor = PLACEHOLDERElevatorConstants.MASTER_MOTOR,
            followerMotor = PLACEHOLDERElevatorConstants.FOLLOWER_MOTOR;
    private final MotionMagicVoltage voltageRequest = new MotionMagicVoltage(0).withEnableFOC(PLACEHOLDERElevatorConstants.FOC_ENABLED);

    @Override
    protected void updateInputs(ElevatorInputsAutoLogged inputs) {
        inputs.masterMotorVoltage = PLACEHOLDERElevatorConstants.MASTER_MOTOR_VOLTAGE_STATUS_SIGNAL.refresh().getValue();
        inputs.followerMotorVoltage = PLACEHOLDERElevatorConstants.FOLLOWER_MOTOR_VOLTAGE_STATUS_SIGNAL.refresh().getValue();

        inputs.masterMotorPositionMeters = getEncoderPositionMeters();
        inputs.followerMotorPositionMeters = getEncoderVelocityMetersPerSecond();

        inputs.masterMotorVelocityMetersPerSecond = getEncoderPositionMeters();
        inputs.followerMotorVelocityMetersPerSecond = getEncoderVelocityMetersPerSecond();
    }

    @Override
    protected void setTargetState(double targetStateMeters) {
        setMotorsVoltage(targetStateMeters);
    }

    @Override
    protected void stopMotors() {
        masterMotor.stopMotor();
        followerMotor.stopMotor();
    }

    private void setMotorsVoltage(double targetStateMeters) {
        masterMotor.setControl(voltageRequest.withPosition(targetStateMeters));
        followerMotor.setControl(voltageRequest.withPosition(targetStateMeters));

    }

    private double getEncoderPositionMeters() {
        return PLACEHOLDERElevatorConstants.ENCODER_POSITION_STATUS_SIGNAL.refresh().getValue();
    }

    private double getEncoderVelocityMetersPerSecond() {
        return PLACEHOLDERElevatorConstants.ENCODER_VELOCITY_STATUS_SIGNAL.refresh().getValue();
    }
}
