package frc.trigon.robot.subsystems.intake;

import com.ctre.phoenix6.controls.VoltageOut;
import edu.wpi.first.wpilibj2.command.button.Trigger;
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
    public void periodic() {
        motor.update();
        updateMechanism();
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

    public Trigger getEarlyNoteCollectionDetectionTrigger() {
        return new Trigger(() -> motor.getSignal(TalonFXSignal.SUPPLY_CURRENT) > IntakeConstants.NOTE_COLLECTION_CURRENT).debounce(IntakeConstants.NOTE_COLLECTION_TIME_THRESHOLD_SECONDS);
    }

    private void updateMechanism() {
        IntakeConstants.MECHANISM.update(motor.getSignal(TalonFXSignal.MOTOR_VOLTAGE));
    }
}