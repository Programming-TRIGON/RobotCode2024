package frc.trigon.robot.subsystems.intake.triumphintake;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;
import frc.trigon.robot.subsystems.intake.IntakeIO;
import frc.trigon.robot.subsystems.intake.IntakeInputsAutoLogged;

public class TriumphIntakeIO extends IntakeIO {
    private final TalonFX
            collectionMotor = TriumphIntakeConstants.COLLECTING_MOTOR,
            angleMotor = TriumphIntakeConstants.ANGLE_MOTOR;
    private final VoltageOut
            angleVoltageRequest = new VoltageOut(0).withEnableFOC(TriumphIntakeConstants.FOC_ENABLED),
            collectionVoltageRequest = new VoltageOut(0).withEnableFOC(TriumphIntakeConstants.FOC_ENABLED);
    private final MotionMagicVoltage anglePositionRequest = new MotionMagicVoltage(0).withEnableFOC(TriumphIntakeConstants.FOC_ENABLED);

    @Override
    protected void updateInputs(IntakeInputsAutoLogged inputs) {
        refreshStatusSignals();
        inputs.anglePositionDegrees = getAnglePosition().getDegrees();
        inputs.angleVelocityDegreesPerSecond = getAngleVelocityDegreesPerSecond();
        inputs.angleMotorVoltage = TriumphIntakeConstants.ANGLE_MOTOR_VOLTAGE_SIGNAL.getValue();
        inputs.angleMotorCurrent = TriumphIntakeConstants.ANGLE_MOTOR_CURRENT_SIGNAL.getValue();
        inputs.angleMotorProfiledSetpointDegrees = getAngleProfiledSetpoint().getDegrees();

        inputs.collectionMotorVelocityRevolutionsPerSecond = TriumphIntakeConstants.COLLECTION_MOTOR_VELOCITY_SIGNAL.getValue();
        inputs.collectionMotorVoltage = TriumphIntakeConstants.COLLECTION_MOTOR_VOLTAGE_SIGNAL.getValue();
        inputs.collectionMotorCurrent = TriumphIntakeConstants.COLLECTION_MOTOR_CURRENT_SIGNAL.getValue();
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
        angleMotor.stopMotor();
        collectionMotor.stopMotor();
    }

    @Override
    protected void setBrake(boolean brake) {
        angleMotor.setNeutralMode(brake ? NeutralModeValue.Brake : NeutralModeValue.Coast);
    }

    private Rotation2d getAnglePosition() {
        return Rotation2d.fromRotations(TriumphIntakeConstants.ANGLE_POSITION_SIGNAL.getValue());
    }

    private double getAngleVelocityDegreesPerSecond() {
        return Units.rotationsToDegrees(TriumphIntakeConstants.ANGLE_VELOCITY_SIGNAL.getValue());
    }

    private Rotation2d getAngleProfiledSetpoint() {
        return Rotation2d.fromRotations(TriumphIntakeConstants.ANGLE_MOTOR_PROFILED_SETPOINT_SIGNAL.getValue());
    }

    private void refreshStatusSignals() {
        BaseStatusSignal.refreshAll(
                TriumphIntakeConstants.ANGLE_POSITION_SIGNAL,
                TriumphIntakeConstants.ANGLE_VELOCITY_SIGNAL,
                TriumphIntakeConstants.ANGLE_MOTOR_CURRENT_SIGNAL,
                TriumphIntakeConstants.ANGLE_MOTOR_VOLTAGE_SIGNAL,
                TriumphIntakeConstants.ANGLE_MOTOR_PROFILED_SETPOINT_SIGNAL,
                TriumphIntakeConstants.COLLECTION_MOTOR_VELOCITY_SIGNAL,
                TriumphIntakeConstants.COLLECTION_MOTOR_CURRENT_SIGNAL,
                TriumphIntakeConstants.COLLECTION_MOTOR_VOLTAGE_SIGNAL
        );
    }
}
