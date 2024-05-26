package frc.trigon.robot.subsystems.intake;

import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.trigon.robot.subsystems.MotorSubsystem;
import org.littletonrobotics.junction.Logger;

public class Intake extends MotorSubsystem {
    private final IntakeInputsAutoLogged intakeInputs = new IntakeInputsAutoLogged();
    private final IntakeIO intakeIO = IntakeIO.generateIO();

    public Intake() {
        setName("Intake");
    }

    @Override
    public void periodic() {
        intakeIO.updateInputs(intakeInputs);
        Logger.processInputs("Intake", intakeInputs);
        updateMechanisms();
    }

    @Override
    public void stop() {
        intakeIO.stop();
        IntakeConstants.COLLECTOR_MECHANISM.setTargetVelocity(0);
    }

    public boolean isActive() {
        return !getDefaultCommand().equals(getCurrentCommand());
    }

    void setTargetVoltage(double collectionVoltage) {
        intakeIO.setTargetVoltage(collectionVoltage);
        IntakeConstants.COLLECTOR_MECHANISM.setTargetVelocity(collectionVoltage);
    }

    public Trigger getEarlyNoteCollectionDetectionTrigger() {
        return new Trigger(() -> intakeInputs.current > IntakeConstants.NOTE_COLLECTION_CURRENT).debounce(IntakeConstants.NOTE_COLLECTION_TIME_THRESHOLD_SECONDS);
    }

    private void updateMechanisms() {
        IntakeConstants.COLLECTOR_MECHANISM.updateMechanism(intakeInputs.voltage);
    }
}