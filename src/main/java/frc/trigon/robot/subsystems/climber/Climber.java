package frc.trigon.robot.subsystems.climber;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.units.Units;
import edu.wpi.first.wpilibj.sysid.SysIdRoutineLog;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.trigon.robot.subsystems.MotorSubsystem;
import frc.trigon.robot.utilities.Conversions;
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
    public void updateLog(SysIdRoutineLog log) {
        log.motor("RightClimber")
                .linearPosition(Units.Meters.of(climberInputs.rightMotorPositionMeters))
                .linearVelocity(Units.MetersPerSecond.of(climberInputs.rightMotorVelocityMetersPerSecond))
                .voltage(Units.Volts.of(climberInputs.rightMotorVoltage));
        log.motor("LeftClimber")
                .linearPosition(Units.Meters.of(climberInputs.leftMotorPositionMeters))
                .linearVelocity(Units.MetersPerSecond.of(climberInputs.leftMotorVelocityMetersPerSecond))
                .voltage(Units.Volts.of(climberInputs.leftMotorVoltage));
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

    void setTargetState(ClimberConstants.ClimberState targetState) {
        climberIO.setPositionMeters(targetState.positionMeters);
    }

    private void updateMechanisms() {
        ClimberConstants.RIGHT_MECHANISM_CURRENT_POSITION_LIGAMENT.setLength(climberInputs.rightMotorPositionMeters + ClimberConstants.RETRACTED_CLIMBER_LENGTH_METERS);
        ClimberConstants.LEFT_MECHANISM_CURRENT_POSITION_LIGAMENT.setLength(climberInputs.leftMotorPositionMeters + ClimberConstants.RETRACTED_CLIMBER_LENGTH_METERS);
        ClimberConstants.RIGHT_MECHANISM_TARGET_POSITION_LIGAMENT.setLength(climberInputs.rightMotorProfiledSetpointMeters);
        ClimberConstants.LEFT_MECHANISM_TARGET_POSITION_LIGAMENT.setLength(climberInputs.leftMotorProfiledSetpointMeters);
        Logger.recordOutput("Mechanisms/ClimberMechanism", ClimberConstants.MECHANISM);
        Logger.recordOutput("Poses/Components/RightClimberPose", getRightClimberPose());
        Logger.recordOutput("Poses/Components/LeftClimberPose", getLeftClimberPose());
    }

    private Pose3d getRightClimberPose() {
        return new Pose3d(new Translation3d(), new Rotation3d(getRightClimberPosition().getRadians(), 0, 0));
    }

    private Pose3d getLeftClimberPose() {
        return new Pose3d(new Translation3d(), new Rotation3d(getLeftClimberPosition().getRadians(), 0, 0));
    }

    private Rotation2d getRightClimberPosition() {
        return Rotation2d.fromRotations(Conversions.distanceToRevolutions(climberInputs.rightMotorPositionMeters, ClimberConstants.DIAMETER_METERS));
    }

    private Rotation2d getLeftClimberPosition() {
        return Rotation2d.fromRotations(Conversions.distanceToRevolutions(climberInputs.leftMotorPositionMeters, ClimberConstants.DIAMETER_METERS));
    }
}
