package frc.trigon.robot.subsystems.climber;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.units.Units;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;

public class ClimberConstants {
    static final SysIdRoutine.Config SYSID_CONFIG = new SysIdRoutine.Config(
            Units.Volts.of(1).per(Units.Second.of(1)),
            Units.Volts.of(7),
            null,
            null
    );

    public enum ClimberState {
        LOWERED(Rotation2d.fromDegrees(0), Rotation2d.fromDegrees(0)),
        RAISED(Rotation2d.fromDegrees(40), Rotation2d.fromDegrees(40));

        final Rotation2d averagePosition;
        final Rotation2d differentialPosition;

        ClimberState(Rotation2d averagePosition, Rotation2d differentialPosition) {
            this.averagePosition = averagePosition;
            this.differentialPosition = differentialPosition;
        }
    }
}
