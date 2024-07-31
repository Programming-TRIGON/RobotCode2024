package frc.trigon.robot.subsystems.transporter;

import com.ctre.phoenix6.controls.VoltageOut;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.trigon.robot.constants.OperatorConstants;
import frc.trigon.robot.hardware.misc.simplesensor.SimpleSensor;
import frc.trigon.robot.hardware.phoenix6.talonfx.TalonFXMotor;
import frc.trigon.robot.hardware.phoenix6.talonfx.TalonFXSignal;
import frc.trigon.robot.subsystems.MotorSubsystem;
import frc.trigon.robot.subsystems.ledstrip.LEDStripCommands;
import frc.trigon.robot.subsystems.ledstrip.LEDStripConstants;

import java.awt.*;

public class Transporter extends MotorSubsystem {
    private final TalonFXMotor motor = TransporterConstants.MOTOR;
    private final SimpleSensor beamBreak = TransporterConstants.BEAM_BREAK;
    private final VoltageOut voltageRequest = new VoltageOut(0).withEnableFOC(TransporterConstants.FOC_ENABLED);
    private TransporterConstants.TransporterState targetState = TransporterConstants.TransporterState.STOPPED;

    public Transporter() {
        setName("Transporter");
        configureStoppingNoteCollectionTrigger();
    }

    @Override
    public void stop() {
        motor.stopMotor();
        TransporterConstants.MECHANISM.setTargetVelocity(0);
    }

    @Override
    public void periodic() {
        motor.update();
        beamBreak.updateSensor();
        updateMechanism();
    }

    public boolean isFeeding() {
        return targetState == TransporterConstants.TransporterState.FEEDING ||
                targetState == TransporterConstants.TransporterState.AUTONOMOUS_FEEDING ||
                targetState == TransporterConstants.TransporterState.SCORE_AMP ||
                targetState == TransporterConstants.TransporterState.SCORE_TRAP;
    }

    public boolean isNoteDetected() {
        return beamBreak.getBinaryValue();
    }

    void setTargetState(TransporterConstants.TransporterState targetState) {
        this.targetState = targetState;
        setTargetVoltage(targetState.voltage);
    }

    void setTargetVoltage(double targetVoltage) {
        motor.setControl(voltageRequest.withOutput(targetVoltage));
        TransporterConstants.MECHANISM.setTargetVelocity(targetVoltage);
    }

    private void configureStoppingNoteCollectionTrigger() {
        final Trigger trigger = new Trigger(beamBreak::getBinaryValue).debounce(TransporterConstants.NOTE_COLLECTION_THRESHOLD_SECONDS);
        trigger.whileTrue(new InstantCommand(() -> {
                    if (!isCollecting() || this.getCurrentCommand() == null)
                        return;
                    if (DriverStation.isAutonomous()) {
                        motor.stopMotor();
                    } else {
                        this.getCurrentCommand().cancel();
                        OperatorConstants.DRIVER_CONTROLLER.rumble(TransporterConstants.NOTE_COLLECTION_RUMBLE_DURATION_SECONDS, TransporterConstants.NOTE_COLLECTION_RUMBLE_POWER);
                    }
                    LEDStripCommands.getAnimateStrobeCommand(Color.orange, 0.1, LEDStripConstants.LED_STRIPS).withTimeout(TransporterConstants.NOTE_COLLECTION_RUMBLE_DURATION_SECONDS).schedule();
                })
        );
    }

    private boolean isCollecting() {
        return targetState == TransporterConstants.TransporterState.COLLECTING;
    }

    private void updateMechanism() {
        TransporterConstants.MECHANISM.update(motor.getSignal(TalonFXSignal.MOTOR_VOLTAGE));
    }
}

