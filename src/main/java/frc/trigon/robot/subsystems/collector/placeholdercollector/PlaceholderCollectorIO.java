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
    private final VoltageOut voltageRequest = new VoltageOut(0).withEnableFOC(PlaceholderCollectorConstants.FOC_ENABLED);
    private final MotionMagicVoltage motionMagicRequest = new MotionMagicVoltage(0).withEnableFOC(PlaceholderCollectorConstants.FOC_ENABLED);

    @Override
    protected void updateInputs(CollectorInputsAutoLogged inputs) {
        BaseStatusSignal.refreshAll();
        inputs.angleMotorPositionDegrees = getAngleMotorPosition().getDegrees();
        inputs.angleMotorVelocityDegreesPerSecond = getAngleMotorVelocityDegreesPerSecond();
        inputs.angleMotorVoltage = angleMotor.getMotorVoltage().getValue();
        inputs.angleMotorCurrent = angleMotor.getSupplyCurrent().getValue();

        inputs.collectionMotorVoltage = collectionMotor.getMotorVoltage().getValue();
        inputs.collectionMotorCurrent = collectionMotor.getSupplyCurrent().getValue();
    }

    @Override
    protected void setCollectionVoltage(double voltage) {
        collectionMotor.setControl(voltageRequest.withOutput(voltage));
    }

    @Override
    protected void setAngleMotorVoltage(double voltage) {
        angleMotor.setControl(voltageRequest.withOutput(voltage));
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
        return Rotation2d.fromRotations(PlaceholderCollectorConstants.COLLECTOR_POSITION_SIGNAL.getValue());
    }

    private double getAngleMotorVelocityDegreesPerSecond() {
        return Units.rotationsToDegrees(PlaceholderCollectorConstants.COLLECTOR_VELOCITY_SIGNAL.getValue());
    }
}
