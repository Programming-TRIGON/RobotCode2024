package frc.trigon.robot.subsystems.collector.simulationcollector;

import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;
import frc.trigon.robot.simulation.SimpleMotorSimulation;
import frc.trigon.robot.simulation.SingleJointedArmSimulation;
import frc.trigon.robot.subsystems.collector.CollectorIO;
import frc.trigon.robot.subsystems.collector.CollectorInputsAutoLogged;

public class SimulationCollectorIO extends CollectorIO {
    private final SingleJointedArmSimulation angleMotor = SimulationCollectorConstants.ANGLE_MOTOR;
    private final SimpleMotorSimulation collectionMotor = SimulationCollectorConstants.COLLECTION_MOTOR;
    private final VoltageOut
            angleVoltageRequest = new VoltageOut(0),
            collectionVoltageRequest = new VoltageOut(0);
    private final MotionMagicVoltage motionMagicRequest = new MotionMagicVoltage(0);

    @Override
    protected void updateInputs(CollectorInputsAutoLogged inputs) {
        inputs.angleMotorPositionDegrees = Units.rotationsToDegrees(angleMotor.getPosition());
        inputs.angleMotorVelocityDegreesPerSecond = Units.rotationsToDegrees(angleMotor.getVelocity());
        inputs.angleMotorVoltage = angleMotor.getVoltage();
        inputs.angleMotorCurrent = angleMotor.getCurrent();
        inputs.angleMotorProfiledSetpointDegrees = Units.rotationsToDegrees(angleMotor.getProfiledSetpoint());

        inputs.collectionMotorVelocityRevolutionsPerSecond = Units.rotationsToDegrees(collectionMotor.getVelocity());
        inputs.collectionMotorVoltage = angleMotor.getVoltage();
        inputs.collectionMotorCurrent = angleMotor.getCurrent();
    }

    @Override
    protected void setCollectionVoltage(double voltage) {
        collectionMotor.setControl(collectionVoltageRequest.withOutput(voltage));
    }

    @Override
    protected void setAngleMotorVoltage(double voltage) {
        angleMotor.setControl(angleVoltageRequest.withOutput(voltage));
    }

    @Override
    protected void setTargetAngle(Rotation2d targetAngle) {
        angleMotor.setControl(motionMagicRequest.withPosition(targetAngle.getRotations()));
    }

    @Override
    protected void stopCollectionMotor() {
        collectionMotor.stop();
    }

    @Override
    protected void stopAngleMotor() {
        angleMotor.stop();
    }
}
