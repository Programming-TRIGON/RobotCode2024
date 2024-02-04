package frc.trigon.robot.subsystems.climber.placeholderclimber;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.controls.DynamicMotionMagicVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;
import frc.trigon.robot.subsystems.climber.ClimberConstants;
import frc.trigon.robot.subsystems.climber.ClimberIO;
import frc.trigon.robot.subsystems.climber.ClimberInputsAutoLogged;
import frc.trigon.robot.utilities.Conversions;

public class PlaceholderClimberIO extends ClimberIO {
    private final TalonFX
            masterMotor = PlaceholderClimberConstants.MASTER_MOTOR,
            followerMotor = PlaceholderClimberConstants.FOLLOWER_MOTOR;
    private final DynamicMotionMagicVoltage
            nonClimbingPositionRequest = new DynamicMotionMagicVoltage(0, PlaceholderClimberConstants.MAX_NON_CLIMBING_VELOCITY, PlaceholderClimberConstants.MAX_NON_CLIMBING_ACCELERATION, 0).withSlot(PlaceholderClimberConstants.NON_CLIMBING_SLOT).withEnableFOC(PlaceholderClimberConstants.ENABLE_FOC),
            climbingPositionRequest = new DynamicMotionMagicVoltage(0, PlaceholderClimberConstants.MAX_CLIMBING_VELOCITY, PlaceholderClimberConstants.MAX_CLIMBING_ACCELERATION, 0).withSlot(PlaceholderClimberConstants.CLIMBING_SLOT).withEnableFOC(PlaceholderClimberConstants.ENABLE_FOC);
    private final VoltageOut voltageRequest = new VoltageOut(0);

    @Override
    protected void updateInputs(ClimberInputsAutoLogged inputs) {
        refreshStatusSignals();
        inputs.encoderPositionMeters = Conversions.revolutionsToDistance(PlaceholderClimberConstants.ENCODER_POSITION_SIGNAL.getValue(), ClimberConstants.DIAMETER_METERS);
        inputs.encoderVelocityMetersPerSecond = Conversions.revolutionsToDistance(PlaceholderClimberConstants.ENCODER_VELOCITY_SIGNAL.getValue(), ClimberConstants.DIAMETER_METERS);
        inputs.motorProfiledSetpointMeters = Conversions.revolutionsToDistance(PlaceholderClimberConstants.MOTOR_SETPOINT_SIGNAL.getValue(), ClimberConstants.DIAMETER_METERS);
        inputs.motorVoltage = PlaceholderClimberConstants.MOTOR_VOLTAGE_SIGNAL.getValue();
        inputs.motorCurrent = PlaceholderClimberConstants.MOTOR_CURRENT_SIGNAL.getValue();
    }

    @Override
    protected void setTargetPositionMeters(double targetPositionMeters, boolean affectedByWeight) {
        final double targetPositionRevolutions = Conversions.distanceToRevolutions(targetPositionMeters, ClimberConstants.DIAMETER_METERS);
        masterMotor.setControl(determineRequest(affectedByWeight).withPosition(targetPositionRevolutions));
    }

    @Override
    protected void setTargetVoltage(double targetVoltage) {
        masterMotor.setControl(voltageRequest.withOutput(targetVoltage));
    }

    @Override
    protected void stop() {
        masterMotor.stopMotor();
    }

    @Override
    protected void setBrake(boolean brake) {
        masterMotor.setNeutralMode(brake ? NeutralModeValue.Brake : NeutralModeValue.Coast);
        followerMotor.setNeutralMode(brake ? NeutralModeValue.Brake : NeutralModeValue.Coast);
    }

    private DynamicMotionMagicVoltage determineRequest(boolean affectedByWeight) {
        return affectedByWeight ? climbingPositionRequest : nonClimbingPositionRequest;
    }

    private void refreshStatusSignals() {
        BaseStatusSignal.refreshAll(
                PlaceholderClimberConstants.ENCODER_POSITION_SIGNAL,
                PlaceholderClimberConstants.ENCODER_VELOCITY_SIGNAL,
                PlaceholderClimberConstants.MOTOR_SETPOINT_SIGNAL,
                PlaceholderClimberConstants.MOTOR_VOLTAGE_SIGNAL,
                PlaceholderClimberConstants.MOTOR_CURRENT_SIGNAL
        );
    }
}
