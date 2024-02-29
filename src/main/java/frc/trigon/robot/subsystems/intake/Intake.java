package frc.trigon.robot.subsystems.intake;

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

    void setTargetVoltage(double collectionVoltage) {
        intakeIO.setTargetVoltage(collectionVoltage);
        IntakeConstants.COLLECTOR_MECHANISM.setTargetVelocity(collectionVoltage);
    }

    private void updateMechanisms() {
        IntakeConstants.COLLECTOR_MECHANISM.updateMechanism(intakeInputs.voltage);
    }
}