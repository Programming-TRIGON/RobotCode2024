package frc.trigon.robot.subsystems.roller;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.trigon.robot.subsystems.MotorSubsystem;
import org.littletonrobotics.junction.Logger;

public class Roller extends MotorSubsystem {
    private final static Roller INSTANCE = new Roller();
    private final RollerIO rollerIO = RollerIO.generateIO();
    private final RollerInputsAutoLogged rollerInputs = new RollerInputsAutoLogged();
    private RollerConstants.RollerState targetState = RollerConstants.RollerState.STOPPED;

    public static Roller getInstance() {
        return INSTANCE;
    }

    private Roller() {
        setName("Roller");
        configureStoppingNoteCollection();
    }

    @Override
    public void stop() {
        rollerIO.stopMotor();
        RollerConstants.ROLLER_MECHANISM.setTargetVelocity(0);
    }

    @Override
    public void periodic() {
        rollerIO.updateInputs(rollerInputs);
        Logger.processInputs("Roller", rollerInputs);
        updateMechanism();
    }

    void setTargetState(RollerConstants.RollerState targetState) {
        this.targetState = targetState;
        setTargetVelocity(targetState.velocityRevolutionsPerSecond);
    }

    private void setTargetVelocity(double targetVelocityRevolutionsPerSecond) {
        rollerIO.setTargetVelocity(targetVelocityRevolutionsPerSecond);
        RollerConstants.ROLLER_MECHANISM.setTargetVelocity(targetVelocityRevolutionsPerSecond);
    }

    private void configureStoppingNoteCollection() {
        final Trigger noteCollectedTrigger = new Trigger(() -> !rollerInputs.infraredSensorTriggered && !isCollecting());
        noteCollectedTrigger.onTrue(new InstantCommand(() -> setTargetState(RollerConstants.RollerState.STOPPED)));
    }

    private boolean isCollecting() {
        return this.targetState == RollerConstants.RollerState.COLLECTING;
    }

    private void updateMechanism() {
        RollerConstants.ROLLER_MECHANISM.updateMechanism(rollerInputs.motorVelocityRevolutionsPerSecond);
    }
}

