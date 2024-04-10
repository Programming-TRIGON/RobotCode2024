package frc.trigon.robot.subsystems.pitcher.triumphpitcher;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.controls.MotionMagicExpoVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.math.geometry.Rotation2d;
import frc.trigon.robot.subsystems.pitcher.PitcherConstants;
import frc.trigon.robot.subsystems.pitcher.PitcherIO;
import frc.trigon.robot.subsystems.pitcher.PitcherInputsAutoLogged;
import frc.trigon.robot.utilities.Conversions;

public class TriumphPitcherIO extends PitcherIO {
    private final TalonFX motor = TriumphPitcherConstants.MOTOR;
    private final MotionMagicExpoVoltage positionRequest = new MotionMagicExpoVoltage(0).withEnableFOC(TriumphPitcherConstants.FOC_ENABLED).withOverrideBrakeDurNeutral(true).withUpdateFreqHz(1000);
    private final VoltageOut voltageRequest = new VoltageOut(0).withEnableFOC(TriumphPitcherConstants.FOC_ENABLED);

    @Override
    protected void updateInputs(PitcherInputsAutoLogged inputs) {
        refreshStatusSignals();

        inputs.pitchDegrees = getPitchDegrees();
        inputs.velocityDegreesPerSecond = getVelocityDegreesPerSecond();
        inputs.voltage = TriumphPitcherConstants.VOLTAGE_SIGNAL.getValue();
        inputs.profiledSetpointDegrees = getProfiledSetpointDegrees();
    }

    @Override
    protected void setTargetPitch(Rotation2d pitch) {
        motor.setControl(positionRequest.withPosition(pitch.getRotations()));
    }

    @Override
    protected void setTargetVoltage(double voltage) {
        motor.setControl(voltageRequest.withOutput(voltage));
    }

    @Override
    protected void setBrake(boolean brake) {
        motor.setNeutralMode(brake ? NeutralModeValue.Brake : NeutralModeValue.Coast);
    }

    @Override
    protected void stop() {
        motor.stopMotor();
    }

    private double getPitchDegrees() {
        final double pitchRevolutions = TriumphPitcherConstants.POSITION_SIGNAL.getValue();
        return Conversions.revolutionsToDegrees(pitchRevolutions);
    }

    private double getVelocityDegreesPerSecond() {
        final double velocityRevolutionsPerSecond = TriumphPitcherConstants.VELOCITY_SIGNAL.getValue();
        final double velocityDegreesPerSecond = Conversions.revolutionsToDegrees(velocityRevolutionsPerSecond);
        return Conversions.motorToSystem(velocityDegreesPerSecond, PitcherConstants.GEAR_RATIO);
    }

    private double getProfiledSetpointDegrees() {
        final double profiledSetpointRevolutions = TriumphPitcherConstants.PROFILED_SETPOINT_SIGNAL.refresh().getValue();
        return Conversions.revolutionsToDegrees(profiledSetpointRevolutions);
    }

    private void refreshStatusSignals() {
        BaseStatusSignal.refreshAll(
                TriumphPitcherConstants.POSITION_SIGNAL,
                TriumphPitcherConstants.VELOCITY_SIGNAL,
//                TriumphPitcherConstants.PROFILED_SETPOINT_SIGNAL,
                TriumphPitcherConstants.VOLTAGE_SIGNAL
        );
    }
}
