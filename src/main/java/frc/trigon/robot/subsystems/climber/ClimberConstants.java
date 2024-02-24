package frc.trigon.robot.subsystems.climber;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.units.Units;
import edu.wpi.first.wpilibj.smartdashboard.Mechanism2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismLigament2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismRoot2d;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj.util.Color8Bit;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;

public class ClimberConstants {
    static final double LIMIT_SWITCH_PRESSED_THRESHOLD_SECONDS = 0.2;
    static final double READY_FOR_ELEVATOR_OPENING_MAXIMUM_POSITION_METERS = 0.4;

    public static final double
            DRUM_RADIUS_METERS = 0.02,
            DRUM_DIAMETER_METERS = DRUM_RADIUS_METERS * 2;
    public static final double RETRACTED_CLIMBER_LENGTH_METERS = 0.185;
    public static final double GEAR_RATIO = 19.64;
    static final double TOLERANCE_METERS = 0.01;

    private static final double
            MECHANISM_WIDTH = 1,
            MECHANISM_HEIGHT = 1;
    private static final double
            MECHANISM_ROOT_X = 0.5,
            MECHANISM_ROOT_Y = 0;
    private static final double LIGAMENT_ANGLE = 90;
    private static final double MECHANISM_LINE_WIDTH = 10;
    static final Mechanism2d MECHANISM = new Mechanism2d(MECHANISM_WIDTH, MECHANISM_HEIGHT);
    private static final MechanismRoot2d
            CURRENT_POSITION_ROOT = MECHANISM.getRoot("ClimberRoot", MECHANISM_ROOT_X, MECHANISM_ROOT_Y),
            TARGET_POSITION_ROOT = MECHANISM.getRoot("TargetPositionRoot", MECHANISM_ROOT_X + 0.25, MECHANISM_ROOT_Y);
    private static final Color8Bit MECHANISM_COLOR = new Color8Bit(Color.kRed);
    static final MechanismLigament2d
            CURRENT_POSITION_LIGAMENT = CURRENT_POSITION_ROOT.append(new MechanismLigament2d("ZCurrentPositionLigament", 0, LIGAMENT_ANGLE, MECHANISM_LINE_WIDTH, MECHANISM_COLOR)),
            TARGET_POSITION_LIGAMENT = TARGET_POSITION_ROOT.append(new MechanismLigament2d("TargetPositionLigament", 0, LIGAMENT_ANGLE, MECHANISM_LINE_WIDTH, new Color8Bit(Color.kGray)));

    static final Pose3d CLIMBER_ORIGIN_POINT = new Pose3d(0.16636, 0, 0.118, new Rotation3d(0, edu.wpi.first.math.util.Units.degreesToRadians(-15), 0));
    static final SysIdRoutine.Config SYSID_CONFIG = new SysIdRoutine.Config(
            Units.Volts.of(1.5).per(Units.Second.of(1)),
            Units.Volts.of(8),
            null,
            null
    );

    public enum ClimberState {
        RESTING(0, false),
        CLIMB(0.05, true),
        CLIMBING_PREPARATION(0.6, false);

        final double positionMeters;
        final boolean affectedByWeight;

        ClimberState(double positionMeters, boolean affectedByWeight) {
            this.positionMeters = positionMeters;
            this.affectedByWeight = affectedByWeight;
        }
    }
}
