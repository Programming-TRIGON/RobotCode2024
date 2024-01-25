package frc.trigon.robot.simulation;

import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.ControlRequest;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.sim.TalonFXSimState;
import edu.wpi.first.wpilibj.Notifier;
import frc.trigon.robot.commands.Commands;
import frc.trigon.robot.constants.RobotConstants;

import java.util.ArrayList;
import java.util.List;

/**
 * A wrapper class for the WPILib default simulation classes, that'll act similarly to how the TalonFX motor controller works.
 */
public abstract class MotorSimulation {
    private static final List<MotorSimulation> REGISTERED_SIMULATIONS = new ArrayList<>();

    static {
        Commands.getDelayedCommand(2, () -> new Notifier(MotorSimulation::updateRegisteredSimulations).startPeriodic(RobotConstants.PERIODIC_TIME_SECONDS)).schedule();
    }

    private final TalonFX motor;
    private final TalonFXSimState motorSimState;
    private final StatusSignal<Double> closedLoopReferenceSignal, motorVoltageSignal;

    protected MotorSimulation() {
        REGISTERED_SIMULATIONS.add(this);
        motor = new TalonFX(REGISTERED_SIMULATIONS.size() - 1);
        motorSimState = motor.getSimState();
        motorSimState.setSupplyVoltage(12);
        closedLoopReferenceSignal = motor.getClosedLoopReference();
        motorVoltageSignal = motor.getMotorVoltage();
        closedLoopReferenceSignal.setUpdateFrequency(1.0 / RobotConstants.PERIODIC_TIME_SECONDS);
        motorVoltageSignal.setUpdateFrequency(1.0 / RobotConstants.PERIODIC_TIME_SECONDS);
    }

    private static void updateRegisteredSimulations() {
        for (MotorSimulation motorSimulation : REGISTERED_SIMULATIONS)
            motorSimulation.updateSimulation();
    }

    public void applyConfiguration(TalonFXConfiguration config) {
        motor.getConfigurator().apply(config);
    }

    public void stop() {
        motor.stopMotor();
    }

    public void setControl(ControlRequest request) {
        motor.setControl(request);
    }

    public double getVoltage() {
        return motorVoltageSignal.refresh().getValue();
    }

    public double getProfiledSetpointRevolutions() {
        return closedLoopReferenceSignal.refresh().getValue();
    }

    private void updateSimulation() {
        setInputVoltage(motorSimState.getMotorVoltage());
        updateMotor();
        motorSimState.setRawRotorPosition(getPositionRevolutions());
        motorSimState.setRotorVelocity(getVelocityRevolutionsPerSecond());
    }

    public abstract double getCurrent();

    public abstract double getPositionRevolutions();

    public abstract double getVelocityRevolutionsPerSecond();

    abstract void setInputVoltage(double voltage);

    abstract void updateMotor();
}
