package frc.trigon.robot.subsystems.shooter;

import edu.wpi.first.units.Measure;
import edu.wpi.first.units.Units;
import edu.wpi.first.units.Voltage;
import edu.wpi.first.wpilibj.sysid.SysIdRoutineLog;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.trigon.robot.constants.OperatorConstants;
import frc.trigon.robot.subsystems.MotorSubsystem;
import frc.trigon.robot.utilities.CurrentWatcher;
import frc.trigon.robot.utilities.ShootingCalculations;
import org.littletonrobotics.junction.Logger;

public class Shooter extends MotorSubsystem {
    private final ShootingCalculations shootingCalculations = ShootingCalculations.getInstance();
    private final ShooterInputsAutoLogged shooterInputs = new ShooterInputsAutoLogged();
    private final ShooterIO shooterIO = ShooterIO.generateIO();
    private double targetVelocityRevolutionsPerSecond = 0;

    public Shooter() {
        setName("Shooter");
        configureCurrentWatcher();
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
                .linearPosition(Units.Meters.of(shooterInputs.positionRevolutions))
                .linearVelocity(Units.MetersPerSecond.of(shooterInputs.velocityRevolutionsPerSecond))
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

    public boolean atTargetShootingVelocity() {
        return Math.abs(shooterInputs.velocityRevolutionsPerSecond - targetVelocityRevolutionsPerSecond) < ShooterConstants.TOLERANCE_REVOLUTIONS;
    }

    void shootAtSpeaker() {
        final double targetVelocityRevolutionsPerSecond = shootingCalculations.calculateTargetShootingVelocity();
        setTargetVelocity(targetVelocityRevolutionsPerSecond);
    }

    void setTargetVelocity(double targetVelocityRevolutionsPerSecond) {
//        final double targetVoltage = ShooterConstants.STATE_SPACE_CONTROLLER.calculate(shooterInputs.velocityRevolutionsPerSecond, targetVelocityRevolutionsPerSecond);
        final double targetVoltage = ShooterConstants.FEEDFORWARD.calculate(targetVelocityRevolutionsPerSecond);
        shooterIO.setTargetVoltage(targetVoltage);
        this.targetVelocityRevolutionsPerSecond = targetVelocityRevolutionsPerSecond;
    }

    void resetController() {
        ShooterConstants.STATE_SPACE_CONTROLLER.reset(shooterInputs.velocityRevolutionsPerSecond);
    }

    private void updateMechanism() {
        ShooterConstants.SHOOTING_MECHANISM.updateMechanism(shooterInputs.velocityRevolutionsPerSecond, targetVelocityRevolutionsPerSecond);
    }

    private void configureCurrentWatcher() {
        new CurrentWatcher(
                () -> shooterInputs.current,
                ShooterConstants.SHOOTING_CURRENT,
                ShooterConstants.SHOOTING_TIME_THRESHOLD,
                () -> OperatorConstants.DRIVER_CONTROLLER.rumble(ShooterConstants.SHOOTING_RUMBLE_DURATION_SECONDS, ShooterConstants.SHOOTING_RUMBLE_POWER)
        );
    }
}

