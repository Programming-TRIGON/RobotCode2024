package frc.trigon.robot.subsystems.climber;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.units.Measure;
import edu.wpi.first.units.Units;
import edu.wpi.first.units.Voltage;
import edu.wpi.first.wpilibj.sysid.SysIdRoutineLog;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.trigon.robot.subsystems.MotorSubsystem;
import frc.trigon.robot.utilities.Conversions;
import org.littletonrobotics.junction.Logger;

public class Climber extends MotorSubsystem {
    private final ClimberInputsAutoLogged climberInputs = new ClimberInputsAutoLogged();
    private final ClimberIO climberIO = ClimberIO.generateIO();
    private ClimberConstants.ClimberState targetState = ClimberConstants.ClimberState.RESTING;

    public Climber() {
        setName("Climber");
        configurePositionResettingLimitSwitch();
    }

    @Override
    public void periodic() {
        climberIO.updateInputs(climberInputs);
        Logger.processInputs("Climber", climberInputs);
        updateNetworkTables();
    }

    @Override
    public void drive(Measure<Voltage> voltageMeasure) {
        climberIO.setTargetVoltage(voltageMeasure.in(Units.Volts));
    }

    @Override
    public void updateLog(SysIdRoutineLog log) {
        log.motor("Climber")
                .linearPosition(Units.Meters.of(climberInputs.positionRevolutions))
                .linearVelocity(Units.MetersPerSecond.of(climberInputs.velocityRevolutionsPerSecond))
                .voltage(Units.Volts.of(climberInputs.motorVoltage));
    }

    @Override
    public void setBrake(boolean brake) {
        climberIO.setBrake(brake);
    }

    @Override
    public SysIdRoutine.Config getSysIdConfig() {
        return ClimberConstants.SYSID_CONFIG;
    }

    @Override
    public void stop() {
        climberIO.stop();
    }

    public boolean atTargetState() {
        return Math.abs(getPositionMeters() - targetState.positionMeters) < ClimberConstants.TOLERANCE_METERS;
    }

    public boolean isReadyForElevatorOpening() {
        return getPositionMeters() < ClimberConstants.READY_FOR_ELEVATOR_OPENING_MAXIMUM_POSITION_METERS;
    }

    void setTargetState(ClimberConstants.ClimberState targetState) {
        setTargetPosition(targetState.positionMeters, targetState.affectedByWeight);
        this.targetState = targetState;
    }

    void setTargetPosition(double targetPositionMeters, boolean affectedByWeight) {
        climberIO.setTargetPosition(toRevolutions(targetPositionMeters), affectedByWeight);
    }

    private void updateNetworkTables() {
        updateMechanisms();
        Logger.recordOutput("Climber/PositionMeters", getPositionMeters());
        Logger.recordOutput("Climber/VelocityMeters", toMeters(climberInputs.velocityRevolutionsPerSecond));
    }

    private void updateMechanisms() {
        ClimberConstants.CURRENT_POSITION_LIGAMENT.setLength(toMeters(climberInputs.positionRevolutions) + ClimberConstants.RETRACTED_CLIMBER_LENGTH_METERS);
        ClimberConstants.TARGET_POSITION_LIGAMENT.setLength(toMeters(climberInputs.profiledSetpointRevolutions) + ClimberConstants.RETRACTED_CLIMBER_LENGTH_METERS);
        Logger.recordOutput("Mechanisms/ClimberMechanism", ClimberConstants.MECHANISM);
        Logger.recordOutput("Poses/Components/ClimberPose", getClimberPose());
    }

    private Pose3d getClimberPose() {
        final Transform3d climberTransform = new Transform3d(
                new Translation3d(0, 0, getPositionMeters()),
                new Rotation3d()
        );
        return ClimberConstants.CLIMBER_ORIGIN_POINT.transformBy(climberTransform);
    }

    private double getPositionMeters() {
        return toMeters(climberInputs.positionRevolutions);
    }

    private double toMeters(double revolutions) {
        return Conversions.revolutionsToDistance(revolutions, ClimberConstants.DRUM_DIAMETER_METERS);
    }

    private double toRevolutions(double meters) {
        return Conversions.distanceToRevolutions(meters, ClimberConstants.DRUM_DIAMETER_METERS);
    }

    private void configurePositionResettingLimitSwitch() {
        final Trigger limitSwitchTrigger = new Trigger(() -> climberInputs.limitSwitchPressed);
        limitSwitchTrigger.and(() -> climberInputs.positionRevolutions != 0).debounce(ClimberConstants.LIMIT_SWITCH_PRESSED_THRESHOLD_SECONDS).whileTrue(new InstantCommand(climberIO::resetPosition).repeatedly().ignoringDisable(true));
    }
}
