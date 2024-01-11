package frc.trigon.robot.subsystems.pitcher.placeholderpitcher;

import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.math.geometry.Rotation2d;
import frc.trigon.robot.subsystems.pitcher.PitcherIO;
import frc.trigon.robot.subsystems.pitcher.PitcherInputsAutoLogged;
import frc.trigon.robot.utilities.Conversions;

public class PLACEHOLDERPitcherIO extends PitcherIO {
    private final TalonFX motor = PLACEHOLDERPitcherConstants.MOTOR;
    private final MotionMagicVoltage positionRequest = new MotionMagicVoltage(0);
    private final VoltageOut feedingVoltageRequest = new VoltageOut(0);

    @Override
    protected void updateInputs(PitcherInputsAutoLogged inputs) {
        inputs.pitchDegrees = getPitchDegrees();
        inputs.velocityDegreesPerSecond = getVelocityDegreesPerSecond();
        inputs.voltage = PLACEHOLDERPitcherConstants.VOLTAGE_SIGNAL.refresh().getValue();
        inputs.profiledSetpointDegrees = getProfiledSetpointDegrees();
    }

    @Override
    protected void setTargetPitch(Rotation2d pitch) {
        motor.setControl(positionRequest.withPosition(pitch.getRotations()));
    }

    @Override
    protected void setTargetVoltage(double voltage) {
        motor.setControl(feedingVoltageRequest.withOutput(voltage));
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
        final double pitchRevolutions = PLACEHOLDERPitcherConstants.POSITION_SIGNAL.refresh().getValue();
        return Conversions.revolutionsToDegrees(pitchRevolutions);
    }

    private double getVelocityDegreesPerSecond() {
        final double velocityRevolutionsPerSecond = PLACEHOLDERPitcherConstants.VELOCITY_SIGNAL.refresh().getValue();
        return Conversions.revolutionsToDegrees(velocityRevolutionsPerSecond);
    }

    private double getProfiledSetpointDegrees() {
        final double profiledSetpointRevolutions = PLACEHOLDERPitcherConstants.PROFILED_SETPOINT_SIGNAL.refresh().getValue();
        return Conversions.revolutionsToDegrees(profiledSetpointRevolutions);
    }
}
