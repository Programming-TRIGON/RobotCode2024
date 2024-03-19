package frc.trigon.robot.subsystems.elevator.triumphelevator;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.controls.DynamicMotionMagicVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.trigon.robot.RobotContainer;
import frc.trigon.robot.commands.Commands;
import frc.trigon.robot.subsystems.elevator.ElevatorConstants;
import frc.trigon.robot.subsystems.elevator.ElevatorIO;
import frc.trigon.robot.subsystems.elevator.ElevatorInputsAutoLogged;
import frc.trigon.robot.utilities.Conversions;
import org.littletonrobotics.junction.Logger;

public class TriumphElevatorIO extends ElevatorIO {
    private final TalonFX
            masterMotor = TriumphElevatorConstants.MASTER_MOTOR,
            followerMotor = TriumphElevatorConstants.FOLLOWER_MOTOR;
    private final DynamicMotionMagicVoltage positionRequest = new DynamicMotionMagicVoltage(0, TriumphElevatorConstants.MOTION_MAGIC_CRUISE_VELOCITY, TriumphElevatorConstants.MOTION_MAGIC_ACCELERATION, 0).withEnableFOC(TriumphElevatorConstants.FOC_ENABLED).withSlot(0);
    private final DynamicMotionMagicVoltage positionWithoutGRequest = new DynamicMotionMagicVoltage(0, TriumphElevatorConstants.MOTION_MAGIC_CRUISE_VELOCITY, TriumphElevatorConstants.MOTION_MAGIC_ACCELERATION, 0).withEnableFOC(TriumphElevatorConstants.FOC_ENABLED).withSlot(1);
    private final VoltageOut voltageRequest = new VoltageOut(0).withEnableFOC(TriumphElevatorConstants.FOC_ENABLED);
    private DynamicMotionMagicVoltage currentPositionRequest = positionWithoutGRequest;
    private double targetPositionRevolutions, speedPercentage;

    public TriumphElevatorIO() {
        Commands.getDelayedCommand(3, () -> {
            var trig = new Trigger(() -> TriumphElevatorConstants.POSITION_SIGNAL.getValue() < 0.07 && RobotContainer.ELEVATOR.isResting());
            trig.onTrue(new InstantCommand(() -> {
                currentPositionRequest = positionWithoutGRequest;
                setTargetPosition(targetPositionRevolutions, speedPercentage);
            }).ignoringDisable(true));
            trig.onFalse(new InstantCommand(() -> {
                currentPositionRequest = positionRequest;
                setTargetPosition(targetPositionRevolutions, speedPercentage);
            }).ignoringDisable(true));
        }).schedule();
    }

    @Override
    protected void updateInputs(ElevatorInputsAutoLogged inputs) {
        refreshStatusSignals();
        if (TriumphElevatorConstants.ENCODER.hasResetOccurred())
            Logger.recordOutput("reset", Timer.getFPGATimestamp());
        Logger.recordOutput("Elevator/Rotor", masterMotor.getRotorPosition().getValue());
        inputs.motorVoltage = TriumphElevatorConstants.MOTOR_VOLTAGE_SIGNAL.getValue();
        inputs.positionRevolutions = TriumphElevatorConstants.POSITION_SIGNAL.getValue();
        inputs.velocityRevolutionsPerSecond = Conversions.motorToSystem(TriumphElevatorConstants.VELOCITY_SIGNAL.getValue(), ElevatorConstants.GEAR_RATIO);
        inputs.profiledSetpointRevolutions = TriumphElevatorConstants.MOTOR_SETPOINT_SIGNAL.refresh().getValue();
        inputs.supplyCurrent = TriumphElevatorConstants.SUPPLY_CURRENT_SIGNAL.getValue();
    }

    @Override
    protected void setTargetPosition(double targetPositionRevolutions, double speedPercentage) {
        this.targetPositionRevolutions = targetPositionRevolutions;
        this.speedPercentage = speedPercentage;
        masterMotor.setControl(scaleProfile(currentPositionRequest, speedPercentage).withPosition(targetPositionRevolutions));
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
        return profile.withVelocity(TriumphElevatorConstants.MOTION_MAGIC_CRUISE_VELOCITY * (speedPercentage / 100)).withAcceleration(TriumphElevatorConstants.MOTION_MAGIC_ACCELERATION * (speedPercentage / 100));
    }

    private void refreshStatusSignals() {
        BaseStatusSignal.refreshAll(
                TriumphElevatorConstants.POSITION_SIGNAL,
                TriumphElevatorConstants.VELOCITY_SIGNAL,
                TriumphElevatorConstants.MOTOR_VOLTAGE_SIGNAL,
                TriumphElevatorConstants.SUPPLY_CURRENT_SIGNAL
//                TriumphElevatorConstants.MOTOR_SETPOINT_SIGNAL
        );
    }
}
