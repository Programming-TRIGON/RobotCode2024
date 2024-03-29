package frc.trigon.robot.subsystems.climber;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.units.Measure;
import edu.wpi.first.units.Units;
import edu.wpi.first.units.Voltage;
import edu.wpi.first.wpilibj.sysid.SysIdRoutineLog;
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
    }

    @Override
    public void periodic() {
        climberIO.updateInputs(climberInputs);
        Logger.processInputs("Climber", climberInputs);
        updateMechanisms();
    }

    @Override
    public void drive(Measure<Voltage> voltageMeasure) {
        climberIO.setTargetVoltage(voltageMeasure.in(Units.Volts));
    }

    @Override
    public void updateLog(SysIdRoutineLog log) {
        log.motor("Climber")
                .linearPosition(Units.Meters.of(Conversions.distanceToRevolutions(climberInputs.encoderPositionMeters, ClimberConstants.DIAMETER_METERS)))
                .linearVelocity(Units.MetersPerSecond.of(Conversions.distanceToRevolutions(climberInputs.encoderVelocityMetersPerSecond, ClimberConstants.DIAMETER_METERS)))
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
        return Math.abs(climberInputs.encoderPositionMeters - targetState.positionMeters) < ClimberConstants.TOLERANCE_METERS;
    }

    void setTargetState(ClimberConstants.ClimberState targetState) {
        climberIO.setTargetPositionMeters(targetState.positionMeters, targetState.affectedByWeight);
        this.targetState = targetState;
    }

    void setTargetPosition(double targetPositionMeters, boolean affectedByWeight) {
        climberIO.setTargetPositionMeters(targetPositionMeters, affectedByWeight);
    }

    private void updateMechanisms() {
        ClimberConstants.CURRENT_POSITION_LIGAMENT.setLength(climberInputs.encoderPositionMeters + ClimberConstants.RETRACTED_CLIMBER_LENGTH_METERS);
        ClimberConstants.TARGET_POSITION_LIGAMENT.setLength(climberInputs.motorProfiledSetpointMeters + ClimberConstants.RETRACTED_CLIMBER_LENGTH_METERS);
        Logger.recordOutput("Mechanisms/ClimberMechanism", ClimberConstants.MECHANISM);
        Logger.recordOutput("Poses/Components/ClimberPose", getClimberPose());
    }

    private Pose3d getClimberPose() {
        final Transform3d climberTransform = new Transform3d(
                new Translation3d(0, 0, climberInputs.encoderPositionMeters),
                new Rotation3d()
        );
        return ClimberConstants.CLIMBER_ORIGIN_POINT.transformBy(climberTransform);
    }
}
