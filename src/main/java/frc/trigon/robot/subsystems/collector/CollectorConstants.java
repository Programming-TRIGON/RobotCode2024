package frc.trigon.robot.subsystems.collector;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.units.Units;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;

public class CollectorConstants {
    static final SysIdRoutine.Config SYSID_CONFIG = new SysIdRoutine.Config(
            Units.Volts.of(1).per(Units.Second.of(1)),
            Units.Volts.of(7),
            null,
            null
    );

    public enum CollectorState {
        RESTING(0, Rotation2d.fromDegrees(90)),
        COLLECTING(-5, Rotation2d.fromDegrees(0)),
        OPENING(0, Rotation2d.fromDegrees(0));

        final double voltage;
        final Rotation2d angle;

        CollectorState(double voltage, Rotation2d angle) {
            this.voltage = voltage;
            this.angle = angle;
        }
    }
}
