package frc.trigon.robot.subsystems.shooter;

import edu.wpi.first.units.Measure;
import edu.wpi.first.units.Units;
import edu.wpi.first.units.Voltage;
import edu.wpi.first.wpilibj.sysid.SysIdRoutineLog;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.trigon.robot.subsystems.MotorSubsystem;
import org.littletonrobotics.junction.Logger;

public class Shooter extends MotorSubsystem {
    private static final Shooter INSTANCE = new Shooter();
    private final ShooterInputsAutoLogged shooterInputs = new ShooterInputsAutoLogged();
    private final ShooterIO shooterIO = ShooterIO.generateIO();
    private double
            targetTopVelocityRevolutionsPerSecond = 0,
            targetBottomVelocityRevolutionsPerSecond = 0;

    public static Shooter getInstance() {
        return INSTANCE;
    }

    private Shooter() {
        setName("Shooter");
    }

    @Override
    public void stop() {
        shooterIO.stop();
        targetTopVelocityRevolutionsPerSecond = 0;
    }

    @Override
    public void periodic() {
        shooterIO.updateInputs(shooterInputs);
        Logger.processInputs("Shooter", shooterInputs);
        updateMechanism();
    }

    @Override
    public void drive(Measure<Voltage> voltageMeasure) {
        shooterIO.setTargetTopVoltage(voltageMeasure.in(Units.Volts));
        shooterIO.setTargetBottomVoltage(voltageMeasure.in(Units.Volts));
    }

    @Override
    public void updateLog(SysIdRoutineLog log) {
        log.motor("Top")
                .linearPosition(Units.Meters.of(shooterInputs.topPositionRevolutions))
                .linearVelocity(Units.MetersPerSecond.of(shooterInputs.topVelocityRevolutionsPerSecond))
                .voltage(Units.Volts.of(shooterInputs.topVoltage));

        log.motor("Bottom")
                .linearPosition(Units.Meters.of(shooterInputs.bottomPositionRevolutions))
                .linearVelocity(Units.MetersPerSecond.of(shooterInputs.bottomVelocityRevolutionsPerSecond))
                .voltage(Units.Volts.of(shooterInputs.bottomVoltage));
    }

    @Override
    public SysIdRoutine.Config getSysIdConfig() {
        return ShooterConstants.SYS_ID_CONFIG;
    }

    public boolean atTargetShootingVelocity() {
        return Math.abs(shooterInputs.topVelocityRevolutionsPerSecond - targetTopVelocityRevolutionsPerSecond) < ShooterConstants.TOLERANCE_REVOLUTIONS &&
                Math.abs(shooterInputs.bottomVelocityRevolutionsPerSecond - targetBottomVelocityRevolutionsPerSecond) < ShooterConstants.TOLERANCE_REVOLUTIONS;
    }

    void shootAtSpeaker(double distanceToSpeaker) {
        final double targetTopVelocityRevolutionsPerSecond = calculateShootingAtSpeakerTopVelocity(distanceToSpeaker);
        final double targetBottomVelocityRevolutionsPerSecond = targetTopVelocityRevolutionsPerSecond * ShooterConstants.TOP_TO_BOTTOM_SHOOTING_RATIO;
        setTargetVelocity(targetTopVelocityRevolutionsPerSecond, targetBottomVelocityRevolutionsPerSecond);
    }

    void setTargetVelocity(double targetTopVelocityRevolutionsPerSecond, double targetBottomVelocityRevolutionsPerSecond) {
        shooterIO.setTargetTopVelocity(targetTopVelocityRevolutionsPerSecond);
        shooterIO.setTargetBottomVelocity(targetBottomVelocityRevolutionsPerSecond);

        ShooterConstants.TOP_SHOOTING_MECHANISM.setTargetVelocity(targetTopVelocityRevolutionsPerSecond);
        ShooterConstants.BOTTOM_SHOOTING_MECHANISM.setTargetVelocity(targetBottomVelocityRevolutionsPerSecond);

        this.targetTopVelocityRevolutionsPerSecond = targetTopVelocityRevolutionsPerSecond;
        this.targetBottomVelocityRevolutionsPerSecond = targetBottomVelocityRevolutionsPerSecond;
    }

    private double calculateShootingAtSpeakerTopVelocity(double distanceToSpeaker) {
        return ShooterConstants.VELOCITY_INTERPOLATION.predict(distanceToSpeaker);
    }

    private void updateMechanism() {
        ShooterConstants.TOP_SHOOTING_MECHANISM.updateMechanism(shooterInputs.topVelocityRevolutionsPerSecond);
        ShooterConstants.BOTTOM_SHOOTING_MECHANISM.updateMechanism(shooterInputs.bottomVelocityRevolutionsPerSecond);
    }
}

