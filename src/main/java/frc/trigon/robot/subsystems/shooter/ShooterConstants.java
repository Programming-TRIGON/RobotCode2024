package frc.trigon.robot.subsystems.shooter;

import edu.wpi.first.units.Units;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.trigon.robot.utilities.SpeedMechanism2d;

public class ShooterConstants {
    static final double TOLERANCE_REVOLUTIONS = 2;
    static final double CURRENT_LIMITING_MINIMUM_DISTANCE_METERS = 4;

    static final SysIdRoutine.Config SYS_ID_CONFIG = new SysIdRoutine.Config(
            Units.Volts.of(0.25).per(Units.Second),
            Units.Volts.of(7),
            Units.Second.of(1000000000)
    );

    private static final double MAX_DISPLAYABLE_VELOCITY = 120;
    static final SpeedMechanism2d SHOOTING_MECHANISM = new SpeedMechanism2d("Mechanisms/ShooterMechanism", MAX_DISPLAYABLE_VELOCITY);

    public static final double GEAR_RATIO = 1;
}
