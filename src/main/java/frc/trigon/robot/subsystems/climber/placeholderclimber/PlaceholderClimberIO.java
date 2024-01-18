package frc.trigon.robot.subsystems.climber.placeholderclimber;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.mechanisms.DifferentialMechanism;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;
import frc.trigon.robot.subsystems.climber.ClimberIO;
import frc.trigon.robot.subsystems.climber.ClimberInputsAutoLogged;

public class PlaceholderClimberIO extends ClimberIO {
    private final TalonFX
            rightMotor = PlaceholderClimberConstants.RIGHT_MOTOR,
            leftMotor = PlaceholderClimberConstants.LEFT_MOTOR;
    private final DifferentialMechanism differentialMechanism = PlaceholderClimberConstants.DIFFERENTIAL_MECHANISM;
    private final MotionMagicVoltage averagePositionRequest = new MotionMagicVoltage(0).withEnableFOC(PlaceholderClimberConstants.ENABLE_FOC);
    private final PositionVoltage differentialPositionRequest = new PositionVoltage(0).withEnableFOC(PlaceholderClimberConstants.ENABLE_FOC);

    @Override
    protected void updateInputs(ClimberInputsAutoLogged inputs) {
        refreshStatusSignals();
        inputs.rightMotorPositionDegrees = Units.rotationsToDegrees(PlaceholderClimberConstants.RIGHT_MOTOR_POSITION_SIGNAL.getValue());
        inputs.rightMotorVelocityDegreesPerSecond = Units.rotationsToDegrees(PlaceholderClimberConstants.RIGHT_MOTOR_VELOCITY_SIGNAL.getValue());
        inputs.rightMotorProfiledSetpointDegrees = Units.rotationsToDegrees(PlaceholderClimberConstants.RIGHT_MOTOR_SETPOINT_SIGNAL.getValue());
        inputs.rightMotorVoltage = PlaceholderClimberConstants.RIGHT_MOTOR_VOLTAGE_SIGNAL.getValue();
        inputs.rightMotorCurrent = PlaceholderClimberConstants.RIGHT_MOTOR_CURRENT_SIGNAL.getValue();

        inputs.leftMotorPositionDegrees = Units.rotationsToDegrees(PlaceholderClimberConstants.LEFT_MOTOR_POSITION_SIGNAL.getValue());
        inputs.leftMotorVelocityDegreesPerSecond = Units.rotationsToDegrees(PlaceholderClimberConstants.LEFT_MOTOR_VELOCITY_SIGNAL.getValue());
        inputs.leftMotorProfiledSetpointDegrees = Units.rotationsToDegrees(PlaceholderClimberConstants.LEFT_MOTOR_SETPOINT_SIGNAL.getValue());
        inputs.leftMotorVoltage = PlaceholderClimberConstants.LEFT_MOTOR_VOLTAGE_SIGNAL.getValue();
        inputs.leftMotorCurrent = PlaceholderClimberConstants.LEFT_MOTOR_CURRENT_SIGNAL.getValue();
    }

    @Override
    protected void setPosition(Rotation2d averagePosition, Rotation2d differentialPosition) {
        differentialMechanism.setControl(averagePositionRequest.withPosition(averagePosition.getRotations()), differentialPositionRequest.withPosition(differentialPosition.getRotations()));
    }

    @Override
    protected void stop() {
        rightMotor.stopMotor();
        leftMotor.stopMotor();
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
