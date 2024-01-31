package frc.trigon.robot.subsystems.roller;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.trigon.robot.commands.Commands;
import frc.trigon.robot.constants.OperatorConstants;
import frc.trigon.robot.subsystems.MotorSubsystem;
import org.littletonrobotics.junction.Logger;

public class Roller extends MotorSubsystem {
    private final RollerIO rollerIO = RollerIO.generateIO();
    private final RollerInputsAutoLogged rollerInputs = new RollerInputsAutoLogged();
    private RollerConstants.RollerState targetState = RollerConstants.RollerState.STOPPED;

    public Roller() {
        setName("Roller");
        Commands.getDelayedCommand(1, this::configureStoppingNoteCollection).schedule();
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

    void setTargetVelocity(double targetVelocityRevolutionsPerSecond) {
        rollerIO.setTargetVelocity(targetVelocityRevolutionsPerSecond);
        RollerConstants.ROLLER_MECHANISM.setTargetVelocity(targetVelocityRevolutionsPerSecond);
    }

    private void configureStoppingNoteCollection() {
        final Trigger noteCollectedTrigger = new Trigger(() -> !rollerInputs.infraredSensorTriggered && !isCollecting());
        noteCollectedTrigger.onTrue(new InstantCommand(() -> {
            setTargetState(RollerConstants.RollerState.STOPPED);
            OperatorConstants.DRIVER_CONTROLLER.rumble(RollerConstants.NOTE_COLLECTION_RUMBLE_DURATION_SECONDS, RollerConstants.NOTE_COLLECTION_RUMBLE_POWER);
        }));
    }

    private boolean isCollecting() {
        return this.targetState == RollerConstants.RollerState.COLLECTING;
    }

    private void updateMechanism() {
        RollerConstants.ROLLER_MECHANISM.updateMechanism(rollerInputs.motorVelocityRevolutionsPerSecond);
    }
}

