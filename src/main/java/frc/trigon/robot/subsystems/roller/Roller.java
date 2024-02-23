package frc.trigon.robot.subsystems.roller;

import edu.wpi.first.wpilibj.DriverStation;
import frc.trigon.robot.RobotContainer;
import frc.trigon.robot.constants.OperatorConstants;
import frc.trigon.robot.subsystems.MotorSubsystem;
import frc.trigon.robot.subsystems.ledstrip.LEDStripCommands;
import frc.trigon.robot.subsystems.ledstrip.LEDStripConstants;
import frc.trigon.robot.utilities.CurrentWatcher;
import org.littletonrobotics.junction.Logger;

import java.awt.*;

public class Roller extends MotorSubsystem {
    private final RollerIO rollerIO = RollerIO.generateIO();
    private final RollerInputsAutoLogged rollerInputs = new RollerInputsAutoLogged();
    private RollerConstants.RollerState targetState = RollerConstants.RollerState.STOPPED;
    private boolean didCollectNote = true;

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
        if (didCollectNote && RobotContainer.SHOOTER.didShootNote())
            didCollectNote = false;
    }

    public boolean didCollectNote() {
        return didCollectNote;
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
                    if (DriverStation.isAutonomous()) {
                        rollerIO.stopMotor();
                    } else {
                        this.getCurrentCommand().cancel();
                        OperatorConstants.DRIVER_CONTROLLER.rumble(RollerConstants.NOTE_COLLECTION_RUMBLE_DURATION_SECONDS, RollerConstants.NOTE_COLLECTION_RUMBLE_POWER);
                    }
                    didCollectNote = true;
                    LEDStripCommands.getAnimateStrobeCommand(Color.orange, 0.1, LEDStripConstants.LED_STRIPS).withTimeout(RollerConstants.NOTE_COLLECTION_RUMBLE_DURATION_SECONDS).schedule();
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

