package frc.trigon.robot.subsystems.collector.simulationcollector;

import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import edu.wpi.first.math.geometry.Rotation2d;
import frc.trigon.robot.motorsimulation.SimpleMotorSimulation;
import frc.trigon.robot.subsystems.collector.CollectorIO;
import frc.trigon.robot.subsystems.collector.CollectorInputsAutoLogged;

public class SimulationCollectorIO extends CollectorIO {
    private final SimpleMotorSimulation
            collectionMotor = SimulationCollectorConstants.COLLECTION_MOTOR,
            angleMotor = SimulationCollectorConstants.ANGLE_MOTOR;
    private final VoltageOut voltageRequest = new VoltageOut(0).withEnableFOC(SimulationCollectorConstants.FOC_ENABLED);
    private final MotionMagicVoltage motionMagicRequest = new MotionMagicVoltage(0).withEnableFOC(SimulationCollectorConstants.FOC_ENABLED);

    @Override
    protected void updateInputs(CollectorInputsAutoLogged inputs) {
        inputs.angleMotorPositionDegrees = angleMotor.getPosition();
        inputs.angleMotorVelocityDegreesPerSecond = angleMotor.getVelocity();
        inputs.angleMotorVoltage = angleMotor.getVoltage();
        inputs.angleMotorCurrent = angleMotor.getCurrent();

        inputs.collectionMotorVoltage = angleMotor.getVoltage();
        inputs.collectionMotorCurrent = angleMotor.getCurrent();
    }

    @Override
    protected void setCollectionVoltage(double voltage) {
        collectionMotor.setControl(voltageRequest.withOutput(voltage));
    }

    @Override
    protected void setTargetAngle(Rotation2d targetAngle) {
        angleMotor.setControl(motionMagicRequest.withPosition(targetAngle.getDegrees()));
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
