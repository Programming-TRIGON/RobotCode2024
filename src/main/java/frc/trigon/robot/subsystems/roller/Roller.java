package frc.trigon.robot.subsystems.roller;

import frc.trigon.robot.constants.OperatorConstants;
import frc.trigon.robot.subsystems.MotorSubsystem;
import frc.trigon.robot.utilities.CurrentWatcher;
import org.littletonrobotics.junction.Logger;

public class Roller extends MotorSubsystem {
    private final RollerIO rollerIO = RollerIO.generateIO();
    private final RollerInputsAutoLogged rollerInputs = new RollerInputsAutoLogged();
    private RollerConstants.RollerState targetState = RollerConstants.RollerState.STOPPED;

    public Roller() {
        setName("Roller");
        configureStoppingNoteCollectionCurrentWatcher();
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
        setTargetVoltage(targetState.voltage);
    }

    void setTargetVoltage(double targetVoltage) {
        rollerIO.setTargetVoltage(targetVoltage);
        RollerConstants.ROLLER_MECHANISM.setTargetVelocity(targetVoltage);
    }

    private void configureStoppingNoteCollectionCurrentWatcher() {
        new CurrentWatcher(
                () -> rollerInputs.motorCurrent,
                RollerConstants.NOTE_COLLECTION_CURRENT,
                RollerConstants.NOTE_COLLECTION_CURRENT_THRESHOLD_SECONDS,
                () -> {
                    if (!isCollecting() || this.getCurrentCommand() == null)
                        return;
                    this.getCurrentCommand().cancel();
                    OperatorConstants.DRIVER_CONTROLLER.rumble(RollerConstants.NOTE_COLLECTION_RUMBLE_DURATION_SECONDS, RollerConstants.NOTE_COLLECTION_RUMBLE_POWER);
                }
        );
    }

    private boolean isCollecting() {
        return targetState == RollerConstants.RollerState.COLLECTING;
    }

    private void updateMechanism() {
        RollerConstants.ROLLER_MECHANISM.updateMechanism(rollerInputs.motorVoltage);
    }
}

