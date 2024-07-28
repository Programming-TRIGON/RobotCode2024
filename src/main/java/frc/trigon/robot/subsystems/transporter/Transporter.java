package frc.trigon.robot.subsystems.transporter;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.trigon.robot.constants.OperatorConstants;
import frc.trigon.robot.subsystems.MotorSubsystem;
import frc.trigon.robot.subsystems.ledstrip.LEDStripCommands;
import frc.trigon.robot.subsystems.ledstrip.LEDStripConstants;
import org.littletonrobotics.junction.Logger;

import java.awt.*;

public class Transporter extends MotorSubsystem {
    private final TransporterIO transporterIO = TransporterIO.generateIO();
    private final TransporterInputsAutoLogged transporterInputs = new TransporterInputsAutoLogged();
    private TransporterConstants.TransporterState targetState = TransporterConstants.TransporterState.STOPPED;

    public Transporter() {
        setName("Transporter");
        configureStoppingNoteCollectionTrigger();
    }

    @Override
    public void stop() {
        transporterIO.stopMotor();
        TransporterConstants.TRANSPORTER_MECHANISM.setTargetVelocity(0);
    }

    @Override
    public void periodic() {
        transporterIO.updateInputs(transporterInputs);
        Logger.processInputs("Transporter", transporterInputs);
        updateMechanism();
    }

    public boolean isFeeding() {
        return targetState == TransporterConstants.TransporterState.FEEDING ||
                targetState == TransporterConstants.TransporterState.AUTONOMOUS_FEEDING ||
                targetState == TransporterConstants.TransporterState.SCORE_AMP ||
                targetState == TransporterConstants.TransporterState.SCORE_TRAP;
    }

    public boolean isNoteDetected() {
        return transporterInputs.sensorTriggered;
    }

    void setTargetState(TransporterConstants.TransporterState targetState) {
        this.targetState = targetState;
        setTargetVoltage(targetState.voltage);
    }

    void setTargetVoltage(double targetVoltage) {
        transporterIO.setTargetVoltage(targetVoltage);
        TransporterConstants.TRANSPORTER_MECHANISM.setTargetVelocity(targetVoltage);
    }

    private void configureStoppingNoteCollectionTrigger() {
        final Trigger trigger = new Trigger(() -> transporterInputs.sensorTriggered).debounce(TransporterConstants.NOTE_COLLECTION_THRESHOLD_SECONDS);
        trigger.whileTrue(new InstantCommand(() -> {
                    if (!isCollecting() || this.getCurrentCommand() == null)
                        return;
                    if (DriverStation.isAutonomous()) {
                        transporterIO.stopMotor();
                    } else {
                        this.getCurrentCommand().cancel();
                        OperatorConstants.DRIVER_CONTROLLER.rumble(TransporterConstants.NOTE_COLLECTION_RUMBLE_DURATION_SECONDS, TransporterConstants.NOTE_COLLECTION_RUMBLE_POWER)
                        ;
                    }
                    LEDStripCommands.getAnimateStrobeCommand(Color.orange, 0.1, LEDStripConstants.LED_STRIPS).withTimeout(TransporterConstants.NOTE_COLLECTION_RUMBLE_DURATION_SECONDS).schedule();
                })
        );
    }

    private boolean isCollecting() {
        return targetState == TransporterConstants.TransporterState.COLLECTING;
    }

    private void updateMechanism() {
        TransporterConstants.TRANSPORTER_MECHANISM.updateMechanism(transporterInputs.motorVoltage);
    }
}

