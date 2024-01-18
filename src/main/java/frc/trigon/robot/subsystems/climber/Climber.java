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

    @Override
    public void periodic() {
        climberIO.updateInputs(climberInputs);
        Logger.processInputs("Climber", climberInputs);
    }

    @Override
    public void updateLog(SysIdRoutineLog log) {
        log.motor("Climber")
                .angularPosition(Units.Degrees.of(climberInputs.rightMotorPositionDegrees))
                .angularVelocity(Units.DegreesPerSecond.of(climberInputs.rightMotorVelocityDegreesPerSecond))
                .voltage(Units.Volts.of(climberInputs.rightMotorVoltage));
    }

    @Override
    public SysIdRoutine.Config getSysIdConfig() {
        return ClimberConstants.SYSID_CONFIG;
    }

    @Override
    public void stop() {
        climberIO.stop();
    }

    void setTargetState(ClimberConstants.ClimberState state) {
        climberIO.setPosition(state.averagePosition, state.differentialPosition);
    }
}
