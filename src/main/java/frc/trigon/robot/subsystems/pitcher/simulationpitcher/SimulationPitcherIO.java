package frc.trigon.robot.subsystems.pitcher.simulationpitcher;

import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import edu.wpi.first.math.geometry.Rotation2d;
import frc.trigon.robot.simulation.SingleJointedArmSimulation;
import frc.trigon.robot.subsystems.pitcher.PitcherIO;
import frc.trigon.robot.subsystems.pitcher.PitcherInputsAutoLogged;
import frc.trigon.robot.utilities.Conversions;

public class SimulationPitcherIO extends PitcherIO {
    private final SingleJointedArmSimulation motor = SimulationPitcherConstants.MOTOR;
    private final MotionMagicVoltage positionRequest = new MotionMagicVoltage(0);
    private final VoltageOut voltageRequest = new VoltageOut(0);

    @Override
    protected void updateInputs(PitcherInputsAutoLogged inputs) {
        inputs.voltage = motor.getVoltage();
        inputs.pitchDegrees = Conversions.revolutionsToDegrees(motor.getPositionRevolutions());
        inputs.velocityDegreesPerSecond = Conversions.revolutionsToDegrees(motor.getVelocityRevolutionsPerSecond());
        inputs.profiledSetpointDegrees = Conversions.revolutionsToDegrees(motor.getProfiledSetpointRevolutions());
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
    protected void stop() {
        motor.stop();
    }
}
