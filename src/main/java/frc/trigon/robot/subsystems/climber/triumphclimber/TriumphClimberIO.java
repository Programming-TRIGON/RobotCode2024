package frc.trigon.robot.subsystems.climber.triumphclimber;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.controls.DynamicMotionMagicVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;
import frc.trigon.robot.subsystems.climber.ClimberIO;
import frc.trigon.robot.subsystems.climber.ClimberInputsAutoLogged;

public class TriumphClimberIO extends ClimberIO {
    private final TalonFX
            masterMotor = TriumphClimberConstants.MASTER_MOTOR,
            followerMotor = TriumphClimberConstants.FOLLOWER_MOTOR;
    private final DynamicMotionMagicVoltage
            nonClimbingPositionRequest = new DynamicMotionMagicVoltage(0, TriumphClimberConstants.MAX_NON_CLIMBING_VELOCITY, TriumphClimberConstants.MAX_NON_CLIMBING_ACCELERATION, 0).withSlot(TriumphClimberConstants.NON_CLIMBING_SLOT).withEnableFOC(TriumphClimberConstants.ENABLE_FOC),
            climbingPositionRequest = new DynamicMotionMagicVoltage(0, TriumphClimberConstants.MAX_CLIMBING_VELOCITY, TriumphClimberConstants.MAX_CLIMBING_ACCELERATION, 0).withSlot(TriumphClimberConstants.CLIMBING_SLOT).withEnableFOC(TriumphClimberConstants.ENABLE_FOC);
    private final VoltageOut voltageRequest = new VoltageOut(0);

    @Override
    protected void updateInputs(ClimberInputsAutoLogged inputs) {
        refreshStatusSignals();

        inputs.positionRevolutions = TriumphClimberConstants.POSITION_SIGNAL.getValue();
        inputs.velocityRevolutionsPerSecond = TriumphClimberConstants.VELOCITY_SIGNAL.getValue();
        inputs.profiledSetpointRevolutions = TriumphClimberConstants.MOTOR_SETPOINT_SIGNAL.refresh().getValue();
        inputs.motorVoltage = TriumphClimberConstants.MOTOR_VOLTAGE_SIGNAL.getValue();
        inputs.motorCurrent = TriumphClimberConstants.MOTOR_CURRENT_SIGNAL.getValue();

        inputs.limitSwitchPressed = !TriumphClimberConstants.LIMIT_SWITCH.get();
    }

    @Override
    protected void resetPosition() {
        masterMotor.setPosition(0);
    }

    @Override
    protected void setTargetPosition(double targetPositionRevolutions, boolean affectedByWeight) {
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
                TriumphClimberConstants.POSITION_SIGNAL,
                TriumphClimberConstants.VELOCITY_SIGNAL,
//                TriumphClimberConstants.MOTOR_SETPOINT_SIGNAL,
                TriumphClimberConstants.MOTOR_VOLTAGE_SIGNAL,
                TriumphClimberConstants.MOTOR_CURRENT_SIGNAL
        );
    }
}
