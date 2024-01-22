package frc.trigon.robot.subsystems.climber.placeholderclimber;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.mechanisms.DifferentialMechanism;
import com.ctre.phoenix6.signals.NeutralModeValue;
import frc.trigon.robot.subsystems.climber.ClimberConstants;
import frc.trigon.robot.subsystems.climber.ClimberIO;
import frc.trigon.robot.subsystems.climber.ClimberInputsAutoLogged;
import frc.trigon.robot.utilities.Conversions;

public class PlaceholderClimberIO extends ClimberIO {
    private final TalonFX
            rightMotor = PlaceholderClimberConstants.RIGHT_MOTOR,
            leftMotor = PlaceholderClimberConstants.LEFT_MOTOR;
    private final DifferentialMechanism differentialMechanism = PlaceholderClimberConstants.DIFFERENTIAL_MECHANISM;
    private final MotionMagicVoltage averagePositionRequest = new MotionMagicVoltage(0).withEnableFOC(PlaceholderClimberConstants.ENABLE_FOC).withSlot(PlaceholderClimberConstants.AVERAGE_SLOT);
    private final PositionVoltage differentialPositionRequest = new PositionVoltage(0).withEnableFOC(PlaceholderClimberConstants.ENABLE_FOC).withSlot(PlaceholderClimberConstants.DIFFERENTIAL_SLOT);

    @Override
    protected void updateInputs(ClimberInputsAutoLogged inputs) {
        refreshStatusSignals();
        inputs.rightMotorPositionMeters = Conversions.revolutionsToDistance(PlaceholderClimberConstants.RIGHT_MOTOR_POSITION_SIGNAL.getValue(), ClimberConstants.DIAMETER_METERS);
        inputs.rightMotorVelocityMetersPerSecond = Conversions.revolutionsToDistance(PlaceholderClimberConstants.RIGHT_MOTOR_VELOCITY_SIGNAL.getValue(), ClimberConstants.DIAMETER_METERS);
        inputs.rightMotorProfiledSetpointMeters = Conversions.revolutionsToDistance(PlaceholderClimberConstants.RIGHT_MOTOR_SETPOINT_SIGNAL.getValue(), ClimberConstants.DIAMETER_METERS);
        inputs.rightMotorVoltage = PlaceholderClimberConstants.RIGHT_MOTOR_VOLTAGE_SIGNAL.getValue();
        inputs.rightMotorCurrent = PlaceholderClimberConstants.RIGHT_MOTOR_CURRENT_SIGNAL.getValue();

        inputs.leftMotorPositionMeters = Conversions.revolutionsToDistance(PlaceholderClimberConstants.LEFT_MOTOR_POSITION_SIGNAL.getValue(), ClimberConstants.DIAMETER_METERS);
        inputs.leftMotorVelocityMetersPerSecond = Conversions.revolutionsToDistance(PlaceholderClimberConstants.LEFT_MOTOR_VELOCITY_SIGNAL.getValue(), ClimberConstants.DIAMETER_METERS);
        inputs.leftMotorProfiledSetpointMeters = Conversions.revolutionsToDistance(PlaceholderClimberConstants.LEFT_MOTOR_SETPOINT_SIGNAL.getValue(), ClimberConstants.DIAMETER_METERS);
        inputs.leftMotorVoltage = PlaceholderClimberConstants.LEFT_MOTOR_VOLTAGE_SIGNAL.getValue();
        inputs.leftMotorCurrent = PlaceholderClimberConstants.LEFT_MOTOR_CURRENT_SIGNAL.getValue();
    }

    @Override
    protected void setPositionMeters(double averagePositionMeters) {
        differentialMechanism.setControl(averagePositionRequest.withPosition(Conversions.distanceToRevolutions(averagePositionMeters, ClimberConstants.DIAMETER_METERS)), differentialPositionRequest);
    }

    @Override
    protected void stop() {
        rightMotor.stopMotor();
        leftMotor.stopMotor();
    }

    @Override
    protected void setBrake(boolean brake) {
        rightMotor.setNeutralMode(brake ? NeutralModeValue.Brake : NeutralModeValue.Coast);
        leftMotor.setNeutralMode(brake ? NeutralModeValue.Brake : NeutralModeValue.Coast);
    }

    private void refreshStatusSignals() {
        BaseStatusSignal.refreshAll(
                PlaceholderClimberConstants.RIGHT_MOTOR_POSITION_SIGNAL,
                PlaceholderClimberConstants.RIGHT_MOTOR_VELOCITY_SIGNAL,
                PlaceholderClimberConstants.RIGHT_MOTOR_SETPOINT_SIGNAL,
                PlaceholderClimberConstants.RIGHT_MOTOR_VOLTAGE_SIGNAL,
                PlaceholderClimberConstants.RIGHT_MOTOR_CURRENT_SIGNAL,
                PlaceholderClimberConstants.LEFT_MOTOR_POSITION_SIGNAL,
                PlaceholderClimberConstants.LEFT_MOTOR_VELOCITY_SIGNAL,
                PlaceholderClimberConstants.LEFT_MOTOR_SETPOINT_SIGNAL,
                PlaceholderClimberConstants.LEFT_MOTOR_VOLTAGE_SIGNAL,
                PlaceholderClimberConstants.LEFT_MOTOR_CURRENT_SIGNAL
        );
    }
}
