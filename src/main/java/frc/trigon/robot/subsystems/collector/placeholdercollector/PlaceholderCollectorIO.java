package frc.trigon.robot.subsystems.collector.placeholdercollector;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;
import frc.trigon.robot.subsystems.collector.CollectorIO;
import frc.trigon.robot.subsystems.collector.CollectorInputsAutoLogged;

public class PlaceholderCollectorIO extends CollectorIO {
    private final TalonFX
            collectionMotor = PlaceholderCollectorConstants.COLLECTING_MOTOR,
            angleMotor = PlaceholderCollectorConstants.ANGLE_MOTOR;
    private final VoltageOut
            angleVoltageRequest = new VoltageOut(0).withEnableFOC(PlaceholderCollectorConstants.FOC_ENABLED),
            collectionVoltageRequest = new VoltageOut(0).withEnableFOC(PlaceholderCollectorConstants.FOC_ENABLED);
    private final MotionMagicVoltage motionMagicRequest = new MotionMagicVoltage(0).withEnableFOC(PlaceholderCollectorConstants.FOC_ENABLED);

    @Override
    protected void updateInputs(CollectorInputsAutoLogged inputs) {
        refreshStatusSignals();
        inputs.angleMotorPositionDegrees = getAngleMotorPosition().getDegrees();
        inputs.angleMotorVelocityDegreesPerSecond = getAngleMotorVelocityDegreesPerSecond();
        inputs.angleMotorVoltage = PlaceholderCollectorConstants.ANGLE_MOTOR_VOLTAGE_SIGNAL.getValue();
        inputs.angleMotorCurrent = PlaceholderCollectorConstants.ANGLE_MOTOR_CURRENT_SIGNAL.getValue();
        inputs.angleMotorProfiledSetPointDegrees = getAngleProfiledSetPoint().getDegrees();

        inputs.collectionMotorVelocityRevolutionsPerSecond = PlaceholderCollectorConstants.COLLECTION_MOTOR_VELOCITY_SIGNAL.getValue();
        inputs.collectionMotorVoltage = PlaceholderCollectorConstants.COLLECTION_MOTOR_VOLTAGE_SIGNAL.getValue();
        inputs.collectionMotorCurrent = PlaceholderCollectorConstants.COLLECTION_MOTOR_CURRENT_SIGNAL.getValue();
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
    protected void stopAngleMotor() {
        collectionMotor.stopMotor();
    }

    @Override
    protected void stopCollectionMotor() {
        angleMotor.stopMotor();
    }

    @Override
    protected void setBrake(boolean brake) {
        angleMotor.setNeutralMode(brake ? NeutralModeValue.Brake : NeutralModeValue.Coast);
    }

    private Rotation2d getAngleMotorPosition() {
        return Rotation2d.fromRotations(PlaceholderCollectorConstants.ANGLE_MOTOR_POSITION_SIGNAL.getValue());
    }

    private double getAngleMotorVelocityDegreesPerSecond() {
        return Units.rotationsToDegrees(PlaceholderCollectorConstants.ANGLE_MOTOR_VELOCITY_SIGNAL.getValue());
    }

    private Rotation2d getAngleProfiledSetPoint() {
        return Rotation2d.fromRotations(PlaceholderCollectorConstants.ANGLE_MOTOR_PROFILED_SET_POINT_SIGNAL.getValue());
    }

    private void refreshStatusSignals() {
        BaseStatusSignal.refreshAll(
                PlaceholderCollectorConstants.ANGLE_MOTOR_POSITION_SIGNAL,
                PlaceholderCollectorConstants.ANGLE_MOTOR_VELOCITY_SIGNAL,
                PlaceholderCollectorConstants.ANGLE_MOTOR_CURRENT_SIGNAL,
                PlaceholderCollectorConstants.ANGLE_MOTOR_VOLTAGE_SIGNAL,
                PlaceholderCollectorConstants.ANGLE_MOTOR_PROFILED_SET_POINT_SIGNAL,
                PlaceholderCollectorConstants.COLLECTION_MOTOR_VELOCITY_SIGNAL,
                PlaceholderCollectorConstants.COLLECTION_MOTOR_CURRENT_SIGNAL,
                PlaceholderCollectorConstants.COLLECTION_MOTOR_VOLTAGE_SIGNAL
        );
    }
}
