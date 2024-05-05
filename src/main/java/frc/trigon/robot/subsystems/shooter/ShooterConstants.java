package frc.trigon.robot.subsystems.shooter;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.units.Units;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.trigon.robot.utilities.SpeedMechanism2d;

public class ShooterConstants {
    private static final double WHEEL_DIAMETER_METERS = 0.1016;
    public static final double GEAR_RATIO = 1;
    public static final double REVOLUTIONS_TO_METERS = GEAR_RATIO / (WHEEL_DIAMETER_METERS * Math.PI);
    // TODO: calibrate
    public static final Pose3d ROBOT_RELATIVE_PIVOT_POINT = new Pose3d(-0.025, 0, 0.190, new Rotation3d(0, 0, Math.PI));
    public static final double SHOOTER_LENGTH_METERS = 0.124;

    static final double TOLERANCE_REVOLUTIONS_PER_SECOND = 2;

    static final SysIdRoutine.Config SYS_ID_CONFIG = new SysIdRoutine.Config(
            Units.Volts.of(0.25).per(Units.Second),
            Units.Volts.of(7),
            Units.Second.of(1000000000)
    );

    private static final double MAX_DISPLAYABLE_VELOCITY = 120;
    static final SpeedMechanism2d SHOOTING_MECHANISM = new SpeedMechanism2d("Mechanisms/ShooterMechanism", MAX_DISPLAYABLE_VELOCITY);
}
