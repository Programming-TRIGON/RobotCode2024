package frc.trigon.robot.subsystems.intake;

import com.ctre.phoenix6.controls.VoltageOut;
import frc.trigon.robot.subsystems.MotorSubsystem;
import org.trigon.hardware.phoenix6.talonfx.TalonFXMotor;
import org.trigon.hardware.phoenix6.talonfx.TalonFXSignal;

public class Intake extends MotorSubsystem {
    private final TalonFXMotor motor = IntakeConstants.MOTOR;
    private final VoltageOut voltageRequest = new VoltageOut(0).withEnableFOC(IntakeConstants.FOC_ENABLED);

    public Intake() {
        setName("Intake");
    }

    @Override
    public void updatePeriodically() {
        motor.update();
    }

    @Override
    public void updateMechanism() {
        IntakeConstants.MECHANISM.update(motor.getSignal(TalonFXSignal.MOTOR_VOLTAGE));
    }

    @Override
    public void stop() {
        motor.stopMotor();
        IntakeConstants.MECHANISM.setTargetVelocity(0);
    }

    public boolean isActive() {
        return !getDefaultCommand().equals(getCurrentCommand());
    }

    void setTargetVoltage(double collectionVoltage) {
        motor.setControl(voltageRequest.withOutput(collectionVoltage));
        IntakeConstants.MECHANISM.setTargetVelocity(collectionVoltage);
    }

    public boolean isEarlyNoteCollectionDetected() {
        return IntakeConstants.EARLY_NOTE_COLLECTION_DETECTION_BOOLEAN_EVENT.getAsBoolean();
    }
}