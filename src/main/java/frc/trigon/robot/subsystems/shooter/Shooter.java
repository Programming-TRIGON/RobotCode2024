package frc.trigon.robot.subsystems.shooter;

import edu.wpi.first.units.Measure;
import edu.wpi.first.units.Units;
import edu.wpi.first.units.Voltage;
import edu.wpi.first.wpilibj.sysid.SysIdRoutineLog;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.trigon.robot.subsystems.MotorSubsystem;
import frc.trigon.robot.subsystems.ledstrip.LEDStripCommands;
import frc.trigon.robot.subsystems.ledstrip.LEDStripConstants;
import frc.trigon.robot.utilities.ShootingCalculations;
import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;

import java.awt.*;

public class Shooter extends MotorSubsystem {
    private final ShootingCalculations shootingCalculations = ShootingCalculations.getInstance();
    private final ShooterInputsAutoLogged shooterInputs = new ShooterInputsAutoLogged();
    private final ShooterIO shooterIO = ShooterIO.generateIO();
    private double targetVelocityRevolutionsPerSecond = 0;

    public Shooter() {
        setName("Shooter");
        configureNoteShootingDetection();
    }

    @Override
    public void stop() {
        shooterIO.stop();
        targetVelocityRevolutionsPerSecond = 0;
    }

    @Override
    public void drive(Measure<Voltage> voltageMeasure) {
        shooterIO.setTargetVoltage(voltageMeasure.in(Units.Volts));
    }

    @Override
    public void updateLog(SysIdRoutineLog log) {
        log.motor("Shooter")
                .angularPosition(Units.Rotations.of(shooterInputs.positionRevolutions))
                .angularVelocity(Units.RotationsPerSecond.of(shooterInputs.velocityRevolutionsPerSecond))
                .voltage(Units.Volts.of(shooterInputs.voltage));
    }

    @Override
    public SysIdRoutine.Config getSysIdConfig() {
        return ShooterConstants.SYS_ID_CONFIG;
    }

    @Override
    public void periodic() {
        shooterIO.updateInputs(shooterInputs);
        Logger.processInputs("Shooter", shooterInputs);
        updateMechanism();
    }

    @AutoLogOutput(key = "Shooter/AtShootingVelocity")
    public boolean atTargetShootingVelocity() {
        return Math.abs(shooterInputs.velocityRevolutionsPerSecond - targetVelocityRevolutionsPerSecond) < ShooterConstants.TOLERANCE_REVOLUTIONS_PER_SECOND;
    }

    public double getTargetVelocityRevolutionsPerSecond() {
        return targetVelocityRevolutionsPerSecond;
    }

    void enableShootingCurrentLimit() {
        shooterIO.enableSupplyCurrentLimit();
    }

    void disableShootingCurrentLimit() {
        shooterIO.disableSupplyCurrentLimit();
    }

    void shootAtSpeaker() {
        final double targetVelocityRevolutionsPerSecond = shootingCalculations.calculateTargetShootingVelocity();
        setTargetVelocity(targetVelocityRevolutionsPerSecond);
    }

    void setTargetVelocity(double targetVelocityRevolutionsPerSecond) {
        shooterIO.setTargetVelocity(targetVelocityRevolutionsPerSecond);
        this.targetVelocityRevolutionsPerSecond = targetVelocityRevolutionsPerSecond;
    }

    private void updateMechanism() {
        ShooterConstants.SHOOTING_MECHANISM.updateMechanism(shooterInputs.velocityRevolutionsPerSecond, targetVelocityRevolutionsPerSecond);
    }

    private void configureNoteShootingDetection() {
        // TODO: find out why !getCurrentCommand().equals(getDefaultCommand()). Not at comp tho
        final Trigger shootingNoteTrigger = new Trigger(() -> shooterInputs.acceleration < -13 && shooterInputs.current > 30 && !getDefaultCommand().equals(getCurrentCommand())).debounce(0.05);
        shootingNoteTrigger.onTrue(LEDStripCommands.getAnimateStrobeCommand(Color.green, 0.1, LEDStripConstants.LED_STRIPS).withTimeout(0.6));
    }
}

