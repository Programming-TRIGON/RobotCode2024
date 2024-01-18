package frc.trigon.robot.subsystems.climber.simulationclimber;

import com.ctre.phoenix6.controls.MotionMagicVoltage;
import edu.wpi.first.math.geometry.Rotation2d;
import frc.trigon.robot.simulation.SimpleMotorSimulation;
import frc.trigon.robot.subsystems.climber.ClimberIO;
import frc.trigon.robot.subsystems.climber.ClimberInputsAutoLogged;

public class SimulationClimberIO extends ClimberIO {
    private final SimpleMotorSimulation
            rightMotor = SimulationClimberConstants.RIGHT_MOTOR,
            leftMotor = SimulationClimberConstants.LEFT_MOTOR;
    private final MotionMagicVoltage positionRequest = new MotionMagicVoltage(0);

    @Override
    protected void updateInputs(ClimberInputsAutoLogged inputs) {

    }

    @Override
    protected void setPosition(Rotation2d averagePosition, Rotation2d differentialPosition) {
        rightMotor.setControl(positionRequest.withPosition(averagePosition.getRotations()));
        leftMotor.setControl(positionRequest.withPosition(averagePosition.getRotations()));
    }

    @Override
    protected void stop() {
        rightMotor.stop();
        leftMotor.stop();
    }
}
