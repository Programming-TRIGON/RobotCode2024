package frc.trigon.robot.subsystems.shooter;

import edu.wpi.first.units.Measure;
import edu.wpi.first.units.Units;
import edu.wpi.first.units.Voltage;
import edu.wpi.first.wpilibj.sysid.SysIdRoutineLog;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.trigon.robot.subsystems.MotorSubsystem;
import frc.trigon.robot.utilities.ShootingCalculations;
import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;

public class Shooter extends MotorSubsystem {
    private final ShootingCalculations shootingCalculations = ShootingCalculations.getInstance();
    private final ShooterInputsAutoLogged shooterInputs = new ShooterInputsAutoLogged();
    private final ShooterIO shooterIO = ShooterIO.generateIO();
    private double targetVelocityRevolutionsPerSecond = 0;

    public Shooter() {
        setName("Shooter");
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

    @AutoLogOutput(key = "Shooter/AtShootingVelocity")
    public boolean atTargetShootingVelocity() {
        return Math.abs(shooterInputs.velocityRevolutionsPerSecond - targetVelocityRevolutionsPerSecond) < ShooterConstants.TOLERANCE_REVOLUTIONS;
    }

    void limitCurrent() {
//        final double distanceFromSpeaker = shootingCalculations.getDistanceFromSpeaker();
//        if (distanceFromSpeaker < ShooterConstants.CURRENT_LIMITING_MINIMUM_DISTANCE_METERS)
//            shooterIO.disableSupplyCurrentLimit();
//        else
//            shooterIO.enableSupplyCurrentLimit();
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
}

