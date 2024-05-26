package frc.trigon.robot.subsystems.pitcher;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.units.Units;
import edu.wpi.first.wpilibj.smartdashboard.Mechanism2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismLigament2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismRoot2d;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj.util.Color8Bit;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;

public class PitcherConstants {
    public static final double PITCHER_LENGTH_METERS = 0.25;
    public static final double GEAR_RATIO = 352.8;
    public static final Rotation2d DEFAULT_PITCH = Rotation2d.fromDegrees(30);

    static final double PITCH_TOLERANCE_DEGREES = 0.3;
    static final SysIdRoutine.Config SYS_ID_CONFIG = new SysIdRoutine.Config(
            Units.Volts.of(0.6).per(Units.Second),
            Units.Volts.of(6),
            Units.Second.of(1000)
    );

    static final Pose3d PITCHER_ORIGIN_POINT = new Pose3d(-0.025, 0, 0.2563, new Rotation3d());
    static final Mechanism2d PITCHER_MECHANISM = new Mechanism2d(PITCHER_LENGTH_METERS * 2, PITCHER_LENGTH_METERS * 2);
    private static final MechanismRoot2d PITCHER_MECHANISM_ROOT = PITCHER_MECHANISM.getRoot("PitcherRoot", PITCHER_LENGTH_METERS, PITCHER_LENGTH_METERS);
    private static final Color8Bit MECHANISM_COLOR = new Color8Bit(Color.kGreen);
    static final MechanismLigament2d
            PITCHER_LIGAMENT = PITCHER_MECHANISM_ROOT.append(new MechanismLigament2d("ZPitcherLigament", PITCHER_LENGTH_METERS, 0, 10, MECHANISM_COLOR)),
            TARGET_PITCHER_LIGAMENT = PITCHER_MECHANISM_ROOT.append(new MechanismLigament2d("PitcherLigament", PITCHER_LENGTH_METERS, 0, 10, new Color8Bit(Color.kGray)));
}
