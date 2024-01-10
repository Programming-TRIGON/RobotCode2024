package frc.trigon.robot.motorsimulation;

import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.Notifier;
import frc.trigon.robot.constants.RobotConstants;
import frc.trigon.robot.utilities.Commands;
import frc.trigon.robot.utilities.Conversions;

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

    private PositionVoltage positionVoltageRequest = null;
    private MotionMagicVoltage motionMagicRequest = null;
    private MotorSimulationConfiguration config = new MotorSimulationConfiguration();
    private PIDController pidController = new PIDController(config.pidConfigs.kP, config.pidConfigs.kI, config.pidConfigs.kD);
    private ProfiledPIDController profiledPIDController = new ProfiledPIDController(config.pidConfigs.kP, config.pidConfigs.kI, config.pidConfigs.kD, new TrapezoidProfile.Constraints(config.motionMagicConfigs.maxVelocity, config.motionMagicConfigs.maxAcceleration));
    private double voltage = 0;

    protected MotorSimulation() {
        REGISTERED_SIMULATIONS.add(this);
    }

    private static void updateRegisteredSimulations() {
        for (MotorSimulation motorSimulation : REGISTERED_SIMULATIONS)
            motorSimulation.updateSimulation(motorSimulation);
    }

    public void applyConfiguration(MotorSimulationConfiguration config) {
        profiledPIDController = new ProfiledPIDController(config.pidConfigs.kP, config.pidConfigs.kI, config.pidConfigs.kD, new TrapezoidProfile.Constraints(config.motionMagicConfigs.maxVelocity, config.motionMagicConfigs.maxAcceleration));
        pidController = new PIDController(config.pidConfigs.kP, config.pidConfigs.kI, config.pidConfigs.kD);
        this.config = config;
        enablePIDContinuousInput(config.pidConfigs.enableContinuousInput);
    }

    public void stop() {
        positionVoltageRequest = null;
        motionMagicRequest = null;
        setVoltage(0);
    }

    public void setControl(VoltageOut voltageRequest) {
        setVoltage(voltageRequest.Output);
    }

    public void setControl(PositionVoltage positionVoltageRequest) {
        this.positionVoltageRequest = positionVoltageRequest;
        motionMagicRequest = null;
        double voltage = pidController.calculate(getPosition(), positionVoltageRequest.Position);
        setVoltage(voltage);
    }

    public void setControl(MotionMagicVoltage motionMagicRequest) {
        this.motionMagicRequest = motionMagicRequest;
        positionVoltageRequest = null;
        double output = calculateMotionMagicOutput(motionMagicRequest);
        setVoltage(output);
    }

    public void setControl(DutyCycleOut dutyCycleRequest) {
        double voltage = Conversions.compensatedPowerToVoltage(dutyCycleRequest.Output, config.voltageCompensationSaturation);
        setVoltage(voltage);
    }

    public double getProfiledTargetPositionRevolutions() {
        return profiledPIDController.getGoal().position;
    }

    public double getVoltage() {
        return voltage;
    }

    public double getPosition() {
        return getPositionRevolutions() * config.conversionFactor;
    }

    public double getVelocity() {
        return getVelocityRevolutionsPerSecond() * config.conversionFactor;
    }

    private void updateSimulation(MotorSimulation motorSimulation) {
        motorSimulation.updateMotor();
        if (motorSimulation.motionMagicRequest != null)
            motorSimulation.setControl(motorSimulation.motionMagicRequest);
        else if (motorSimulation.positionVoltageRequest != null)
            motorSimulation.setControl(motorSimulation.positionVoltageRequest);
    }

    private double calculateMotionMagicOutput(MotionMagicVoltage motionMagicRequest) {
        double pidOutput = profiledPIDController.calculate(getPosition(), motionMagicRequest.Position);
        double feedforwardOutput = calculateFeedforward(
                config.feedForwardConfigs,
                Units.rotationsToRadians(motionMagicRequest.Position / config.conversionFactor),
                profiledPIDController.getGoal().velocity
        );
        return pidOutput + feedforwardOutput;
    }

    private void setVoltage(double voltage) {
        double compensatedVoltage = MathUtil.clamp(voltage, -config.voltageCompensationSaturation, config.voltageCompensationSaturation);
        this.voltage = compensatedVoltage;
        setInputVoltage(compensatedVoltage);
    }

    private void enablePIDContinuousInput(boolean enableContinuousInput) {
        if (enableContinuousInput) {
            pidController.enableContinuousInput(-0.5 * config.conversionFactor, 0.5 * config.conversionFactor);
            profiledPIDController.enableContinuousInput(-0.5 * config.conversionFactor, 0.5 * config.conversionFactor);
        } else {
            pidController.disableContinuousInput();
            profiledPIDController.disableContinuousInput();
        }
    }

    public abstract double getCurrent();

    /**
     * Calculates the feedforward.
     *
     * @param feedForwardConfiguration the feedforward configuration
     * @param targetPositionRadians    the target position in radians
     * @param targetVelocity           the target velocity in the conversion factor
     * @return The calculated feedforward voltage
     */
    abstract double calculateFeedforward(MotorSimulationConfiguration.FeedForwardConfigs feedForwardConfiguration, double targetPositionRadians, double targetVelocity);

    abstract double getPositionRevolutions();

    abstract double getVelocityRevolutionsPerSecond();

    abstract void setInputVoltage(double voltage);

    abstract void updateMotor();
}
