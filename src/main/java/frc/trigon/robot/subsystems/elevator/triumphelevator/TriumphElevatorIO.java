package frc.trigon.robot.subsystems.elevator.triumphelevator;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;
import frc.trigon.robot.subsystems.elevator.ElevatorIO;
import frc.trigon.robot.subsystems.elevator.ElevatorInputsAutoLogged;

public class TriumphElevatorIO extends ElevatorIO {
    private final TalonFX
            masterMotor = TriumphElevatorConstants.MASTER_MOTOR,
            followerMotor = TriumphElevatorConstants.FOLLOWER_MOTOR;
    private final MotionMagicVoltage positionRequest = new MotionMagicVoltage(0).withEnableFOC(TriumphElevatorConstants.FOC_ENABLED);
    private final VoltageOut voltageRequest = new VoltageOut(0).withEnableFOC(TriumphElevatorConstants.FOC_ENABLED);

    @Override
    protected void updateInputs(ElevatorInputsAutoLogged inputs) {
        refreshStatusSignals();
        inputs.motorVoltage = TriumphElevatorConstants.MOTOR_VOLTAGE_STATUS_SIGNAL.getValue();
        inputs.positionRevolutions = TriumphElevatorConstants.ENCODER_POSITION_STATUS_SIGNAL.getValue();
        inputs.velocityRevolutionsPerSecond = TriumphElevatorConstants.ENCODER_VELOCITY_STATUS_SIGNAL.getValue();
        inputs.profiledSetpointRevolutions = TriumphElevatorConstants.MOTOR_SETPOINT_STATUS_SIGNAL.getValue();
    }

    @Override
    protected void setTargetPosition(double targetPositionRevolutions) {
        masterMotor.setControl(positionRequest.withPosition(targetPositionRevolutions));
    }

    @Override
    protected void setTargetVoltage(double voltage) {
        masterMotor.setControl(voltageRequest.withOutput(voltage));
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

    private void refreshStatusSignals() {
        BaseStatusSignal.refreshAll(
                TriumphElevatorConstants.ENCODER_POSITION_STATUS_SIGNAL,
                TriumphElevatorConstants.ENCODER_VELOCITY_STATUS_SIGNAL,
                TriumphElevatorConstants.MOTOR_VOLTAGE_STATUS_SIGNAL,
                TriumphElevatorConstants.MOTOR_SETPOINT_STATUS_SIGNAL
        );
    }
}
