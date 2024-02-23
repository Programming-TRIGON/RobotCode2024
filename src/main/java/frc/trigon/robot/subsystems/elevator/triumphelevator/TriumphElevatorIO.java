package frc.trigon.robot.subsystems.elevator.triumphelevator;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.controls.DynamicMotionMagicVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;
import frc.trigon.robot.subsystems.elevator.ElevatorConstants;
import frc.trigon.robot.subsystems.elevator.ElevatorIO;
import frc.trigon.robot.subsystems.elevator.ElevatorInputsAutoLogged;
import frc.trigon.robot.utilities.Conversions;

public class TriumphElevatorIO extends ElevatorIO {
    private final TalonFX
            masterMotor = TriumphElevatorConstants.MASTER_MOTOR,
            followerMotor = TriumphElevatorConstants.FOLLOWER_MOTOR;
    private final DynamicMotionMagicVoltage positionRequest = new DynamicMotionMagicVoltage(0, TriumphElevatorConstants.MOTION_MAGIC_CRUISE_VELOCITY, TriumphElevatorConstants.MOTION_MAGIC_ACCELERATION, 0).withEnableFOC(TriumphElevatorConstants.FOC_ENABLED);
    private final VoltageOut voltageRequest = new VoltageOut(0).withEnableFOC(TriumphElevatorConstants.FOC_ENABLED);

    @Override
    protected void updateInputs(ElevatorInputsAutoLogged inputs) {
        refreshStatusSignals();
        inputs.motorVoltage = TriumphElevatorConstants.MOTOR_VOLTAGE_SIGNAL.getValue();
        inputs.positionRevolutions = TriumphElevatorConstants.POSITION_SIGNAL.getValue();
        inputs.velocityRevolutionsPerSecond = Conversions.motorToSystem(TriumphElevatorConstants.VELOCITY_SIGNAL.getValue(), ElevatorConstants.GEAR_RATIO);
        inputs.profiledSetpointRevolutions = TriumphElevatorConstants.MOTOR_SETPOINT_SIGNAL.refresh().getValue();
    }

    @Override
    protected void setTargetPosition(double targetPositionRevolutions) {
        masterMotor.setControl(positionRequest.withPosition(targetPositionRevolutions));
    }

    @Override
    protected void setTargetPosition(double targetPositionRevolutions, double speedPercentage) {
        masterMotor.setControl(scaleProfile(positionRequest, speedPercentage).withPosition(targetPositionRevolutions));
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

    private DynamicMotionMagicVoltage scaleProfile(DynamicMotionMagicVoltage profile, double speedPercentage) {
        return profile.withVelocity(profile.Velocity * speedPercentage).withAcceleration(profile.Acceleration * speedPercentage);
    }

    private void refreshStatusSignals() {
        BaseStatusSignal.refreshAll(
                TriumphElevatorConstants.POSITION_SIGNAL,
                TriumphElevatorConstants.VELOCITY_SIGNAL,
                TriumphElevatorConstants.MOTOR_VOLTAGE_SIGNAL
//                TriumphElevatorConstants.MOTOR_SETPOINT_SIGNAL
        );
    }
}
