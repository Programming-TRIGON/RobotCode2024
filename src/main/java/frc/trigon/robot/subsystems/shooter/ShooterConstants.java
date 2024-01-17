package frc.trigon.robot.subsystems.shooter;

import edu.wpi.first.units.Units;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.trigon.robot.constants.ShootingConstants;
import frc.trigon.robot.utilities.LinearInterpolation;
import frc.trigon.robot.utilities.SpeedMechanism2d;

import java.util.ArrayList;

public class ShooterConstants {
    public static final double SHOOTER_GEAR_RATIO = 1;
    static final double TOP_TO_BOTTOM_SHOOTING_RATIO = 1;

    static final double TOLERANCE_REVOLUTIONS = 0.1;
    static final LinearInterpolation VELOCITY_INTERPOLATION = generateInterpolation();
    static final SysIdRoutine.Config SYS_ID_CONFIG = new SysIdRoutine.Config(
            Units.Volts.of(0.25).per(Units.Second),
            Units.Volts.of(10),
            Units.Second.of(100)
    );

    private static final double MAX_DISPLAYABLE_VELOCITY = 100;
    static final SpeedMechanism2d
            TOP_SHOOTING_MECHANISM = new SpeedMechanism2d("Mechanisms/TopShooterMechanism", MAX_DISPLAYABLE_VELOCITY),
            BOTTOM_SHOOTING_MECHANISM = new SpeedMechanism2d("Mechanisms/BottomShooterMechanism", MAX_DISPLAYABLE_VELOCITY);

    private static LinearInterpolation generateInterpolation() {
        final ArrayList<LinearInterpolation.Point> points = new ArrayList<>();
        for (ShootingConstants.ShootingPosition position : ShootingConstants.SHOOTING_POSITIONS)
            points.add(new LinearInterpolation.Point(position.distanceMeters(), position.shooterVelocityRevolutionsPerSecond()));
        return new LinearInterpolation(points);
    }
}
