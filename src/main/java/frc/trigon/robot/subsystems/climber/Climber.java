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
import org.littletonrobotics.junction.Logger;

public class Climber extends MotorSubsystem {
    private final static Climber INSTANCE = new Climber();
    private final ClimberInputsAutoLogged climberInputs = new ClimberInputsAutoLogged();
    private final ClimberIO climberIO = ClimberIO.generateIO();

    public static Climber getInstance() {
        return INSTANCE;
    }

    private Climber() {
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
        climberIO.setVoltage(voltageMeasure.in(Units.Volts));
    }

    @Override
    public void updateLog(SysIdRoutineLog log) {
        log.motor("Climber")
                .linearPosition(Units.Meters.of(climberInputs.encoderPositionMeters))
                .linearVelocity(Units.MetersPerSecond.of(climberInputs.encoderVelocityMetersPerSecond))
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
        return Math.abs(climberInputs.encoderPositionMeters - climberInputs.motorProfiledSetpointMeters) < ClimberConstants.TOLERANCE_METERS;
    }

    void setTargetState(ClimberConstants.ClimberState targetState) {
        climberIO.setPositionMeters(targetState.positionMeters);
    }

    private void updateMechanisms() {
        ClimberConstants.MECHANISM_CURRENT_POSITION_LIGAMENT.setLength(climberInputs.encoderPositionMeters + ClimberConstants.RETRACTED_CLIMBER_LENGTH_METERS);
        ClimberConstants.MECHANISM_TARGET_POSITION_LIGAMENT.setLength(climberInputs.motorProfiledSetpointMeters + ClimberConstants.RETRACTED_CLIMBER_LENGTH_METERS);
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
