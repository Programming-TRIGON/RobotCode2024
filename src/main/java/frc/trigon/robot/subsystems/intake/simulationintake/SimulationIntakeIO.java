package frc.trigon.robot.subsystems.intake.simulationintake;

import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;
import frc.trigon.robot.simulation.FlywheelSimulation;
import frc.trigon.robot.simulation.SingleJointedArmSimulation;
import frc.trigon.robot.subsystems.intake.IntakeIO;
import frc.trigon.robot.subsystems.intake.IntakeInputsAutoLogged;

public class SimulationIntakeIO extends IntakeIO {
    private final SingleJointedArmSimulation angleMotor = SimulationIntakeConstants.ANGLE_MOTOR;
    private final FlywheelSimulation collectionMotor = SimulationIntakeConstants.COLLECTION_MOTOR;
    private final VoltageOut
            angleVoltageRequest = new VoltageOut(0),
            collectionVoltageRequest = new VoltageOut(0);
    private final MotionMagicVoltage anglePositionRequest = new MotionMagicVoltage(0);

    @Override
    protected void updateInputs(IntakeInputsAutoLogged inputs) {
        inputs.anglePositionDegrees = Units.rotationsToDegrees(angleMotor.getPositionRevolutions());
        inputs.angleVelocityDegreesPerSecond = Units.rotationsToDegrees(angleMotor.getVelocityRevolutionsPerSecond());
        inputs.angleMotorVoltage = angleMotor.getVoltage();
        inputs.angleMotorCurrent = angleMotor.getCurrent();
        inputs.angleMotorProfiledSetpointDegrees = Units.rotationsToDegrees(angleMotor.getProfiledSetpointRevolutions());

        inputs.collectionMotorVelocityRevolutionsPerSecond = collectionMotor.getVelocityRevolutionsPerSecond();
        inputs.collectionMotorVoltage = angleMotor.getVoltage();
        inputs.collectionMotorCurrent = angleMotor.getCurrent();
    }

    @Override
    protected void setTargetCollectionVoltage(double voltage) {
        collectionMotor.setControl(collectionVoltageRequest.withOutput(voltage));
    }

    @Override
    protected void setTargetAngleMotorVoltage(double voltage) {
        angleMotor.setControl(angleVoltageRequest.withOutput(voltage));
    }

    @Override
    protected void setTargetAngle(Rotation2d targetAngle) {
        angleMotor.setControl(anglePositionRequest.withPosition(targetAngle.getRotations()));
    }

    @Override
    protected void stop() {
        angleMotor.stop();
        collectionMotor.stop();
    }
}
