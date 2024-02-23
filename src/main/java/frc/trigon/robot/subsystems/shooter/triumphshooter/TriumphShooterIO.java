package frc.trigon.robot.subsystems.shooter.triumphshooter;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import frc.trigon.robot.subsystems.shooter.ShooterIO;
import frc.trigon.robot.subsystems.shooter.ShooterInputsAutoLogged;

public class TriumphShooterIO extends ShooterIO {
    private final TalonFX masterMotor = TriumphShooterConstants.MASTER_MOTOR;
    private final VelocityVoltage velocityRequest = new VelocityVoltage(0).withEnableFOC(TriumphShooterConstants.FOC_ENABLED);
    private final VoltageOut voltageRequest = new VoltageOut(0).withEnableFOC(TriumphShooterConstants.FOC_ENABLED);

    @Override
    protected void updateInputs(ShooterInputsAutoLogged inputs) {
        refreshStatusSignals();

        inputs.positionRevolutions = TriumphShooterConstants.MASTER_MOTOR_POSITION_SIGNAL.getValue();
        inputs.velocityRevolutionsPerSecond = TriumphShooterConstants.MASTER_MOTOR_VELOCITY_SIGNAL.getValue();
        inputs.voltage = TriumphShooterConstants.MASTER_MOTOR_VOLTAGE_SIGNAL.getValue();
        inputs.current = TriumphShooterConstants.MASTER_MOTOR_CURRENT_SIGNAL.getValue();
        inputs.acceleration = TriumphShooterConstants.MASTER_MOTOR_ACCELERATION.getValue();
    }

    @Override
    protected void enableSupplyCurrentLimit() {
        final CurrentLimitsConfigs config = new CurrentLimitsConfigs();
        config.SupplyCurrentLimit = TriumphShooterConstants.HOLDING_CURRENT;
        config.SupplyCurrentThreshold = TriumphShooterConstants.CURRENT_LIMIT_THRESHOLD;
        config.SupplyTimeThreshold = TriumphShooterConstants.CURRENT_LIMIT_TIME_THRESHOLD;
        config.SupplyCurrentLimitEnable = true;
        masterMotor.getConfigurator().apply(config);

    }

    @Override
    protected void disableSupplyCurrentLimit() {
        final CurrentLimitsConfigs config = new CurrentLimitsConfigs();
        config.SupplyCurrentLimitEnable = false;
        masterMotor.getConfigurator().apply(config);
    }

    @Override
    protected void setTargetVelocity(double targetVelocity) {
        masterMotor.setControl(velocityRequest.withVelocity(targetVelocity));
    }

    @Override
    protected void setTargetVoltage(double targetVoltage) {
        masterMotor.setControl(voltageRequest.withOutput(targetVoltage));
    }

    @Override
    protected void stop() {
        masterMotor.stopMotor();
    }

    private void refreshStatusSignals() {
        BaseStatusSignal.refreshAll(
                TriumphShooterConstants.MASTER_MOTOR_VELOCITY_SIGNAL,
                TriumphShooterConstants.MASTER_MOTOR_POSITION_SIGNAL,
                TriumphShooterConstants.MASTER_MOTOR_VOLTAGE_SIGNAL,
                TriumphShooterConstants.MASTER_MOTOR_CURRENT_SIGNAL,
                TriumphShooterConstants.MASTER_MOTOR_ACCELERATION
        );
    }
}
