package frc.trigon.robot.subsystems.intake.placeholderintake;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;
import frc.trigon.robot.subsystems.intake.IntakeIO;
import frc.trigon.robot.subsystems.intake.IntakeInputsAutoLogged;

public class PlaceholderIntakeIO extends IntakeIO {
    private final TalonFX
            collectionMotor = PlaceholderIntakeConstants.COLLECTING_MOTOR,
            angleMotor = PlaceholderIntakeConstants.ANGLE_MOTOR;
    private final VoltageOut
            angleVoltageRequest = new VoltageOut(0).withEnableFOC(PlaceholderIntakeConstants.FOC_ENABLED),
            collectionVoltageRequest = new VoltageOut(0).withEnableFOC(PlaceholderIntakeConstants.FOC_ENABLED);
    private final MotionMagicVoltage anglePositionRequest = new MotionMagicVoltage(0).withEnableFOC(PlaceholderIntakeConstants.FOC_ENABLED);

    @Override
    protected void updateInputs(IntakeInputsAutoLogged inputs) {
        refreshStatusSignals();
        inputs.anglePositionDegrees = getAnglePosition().getDegrees();
        inputs.angleVelocityDegreesPerSecond = getAngleVelocityDegreesPerSecond();
        inputs.angleMotorVoltage = PlaceholderIntakeConstants.ANGLE_MOTOR_VOLTAGE_SIGNAL.getValue();
        inputs.angleMotorCurrent = PlaceholderIntakeConstants.ANGLE_MOTOR_CURRENT_SIGNAL.getValue();
        inputs.angleMotorProfiledSetpointDegrees = getAngleProfiledSetpoint().getDegrees();

        inputs.collectionMotorVelocityRevolutionsPerSecond = PlaceholderIntakeConstants.COLLECTION_MOTOR_VELOCITY_SIGNAL.getValue();
        inputs.collectionMotorVoltage = PlaceholderIntakeConstants.COLLECTION_MOTOR_VOLTAGE_SIGNAL.getValue();
        inputs.collectionMotorCurrent = PlaceholderIntakeConstants.COLLECTION_MOTOR_CURRENT_SIGNAL.getValue();
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
        return Rotation2d.fromRotations(PlaceholderIntakeConstants.ANGLE_POSITION_SIGNAL.getValue());
    }

    private double getAngleVelocityDegreesPerSecond() {
        return Units.rotationsToDegrees(PlaceholderIntakeConstants.ANGLE_VELOCITY_SIGNAL.getValue());
    }

    private Rotation2d getAngleProfiledSetpoint() {
        return Rotation2d.fromRotations(PlaceholderIntakeConstants.ANGLE_MOTOR_PROFILED_SETPOINT_SIGNAL.getValue());
    }

    private void refreshStatusSignals() {
        BaseStatusSignal.refreshAll(
                PlaceholderIntakeConstants.ANGLE_POSITION_SIGNAL,
                PlaceholderIntakeConstants.ANGLE_VELOCITY_SIGNAL,
                PlaceholderIntakeConstants.ANGLE_MOTOR_CURRENT_SIGNAL,
                PlaceholderIntakeConstants.ANGLE_MOTOR_VOLTAGE_SIGNAL,
                PlaceholderIntakeConstants.ANGLE_MOTOR_PROFILED_SETPOINT_SIGNAL,
                PlaceholderIntakeConstants.COLLECTION_MOTOR_VELOCITY_SIGNAL,
                PlaceholderIntakeConstants.COLLECTION_MOTOR_CURRENT_SIGNAL,
                PlaceholderIntakeConstants.COLLECTION_MOTOR_VOLTAGE_SIGNAL
        );
    }
}
