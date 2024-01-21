package frc.trigon.robot.subsystems.climber;

import edu.wpi.first.units.Units;
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
    public SysIdRoutine.Config getSysIdConfig() {
        return ClimberConstants.SYSID_CONFIG;
    }

    @Override
    public void stop() {
        climberIO.stop();
    }

    void setTargetState(ClimberConstants.ClimberState targetState) {
        climberIO.setTargetPositionMeters(targetState.positionMeters);
    }

    private void updateMechanisms() {
        ClimberConstants.RIGHT_MECHANISM_CURRENT_POSITION_LIGAMENT.setLength(climberInputs.rightMotorPositionMeters);
        ClimberConstants.LEFT_MECHANISM_CURRENT_POSITION_LIGAMENT.setLength(climberInputs.leftMotorPositionMeters);
        ClimberConstants.RIGHT_MECHANISM_TARGET_POSITION_LIGAMENT.setLength(climberInputs.rightMotorProfiledSetpointMeters);
        ClimberConstants.LEFT_MECHANISM_TARGET_POSITION_LIGAMENT.setLength(climberInputs.leftMotorProfiledSetpointMeters);
        Logger.recordOutput("Mechanisms/ClimberMechanism", ClimberConstants.MECHANISM);
    }
}
