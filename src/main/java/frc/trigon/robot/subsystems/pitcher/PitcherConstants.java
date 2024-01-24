package frc.trigon.robot.subsystems.pitcher;

import edu.wpi.first.math.geometry.Pose3d;
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
    public static final double GEAR_RATIO = 342.22;

    static final double PITCH_TOLERANCE_DEGREES = 1;
    static final SysIdRoutine.Config SYS_ID_CONFIG = new SysIdRoutine.Config(
            Units.Volts.of(1).per(Units.Second),
            Units.Volts.of(7),
            null
    );

    static final Pose3d PITCHER_ORIGIN_POINT = new Pose3d(-0.025, 0, 0.2563, new Rotation3d());
    static final Mechanism2d PITCHER_MECHANISM = new Mechanism2d(PITCHER_LENGTH_METERS * 2, PITCHER_LENGTH_METERS * 2, new Color8Bit(Color.kBlack));
    private static final MechanismRoot2d PITCHER_MECHANISM_ROOT = PITCHER_MECHANISM.getRoot("PitcherRoot", PITCHER_LENGTH_METERS, PITCHER_LENGTH_METERS);
    static final MechanismLigament2d
            PITCHER_LIGAMENT = PITCHER_MECHANISM_ROOT.append(new MechanismLigament2d("ZPitcherLigament", PITCHER_LENGTH_METERS, 0, 10, new Color8Bit(Color.kBlue))),
            TARGET_PITCHER_LIGAMENT = PITCHER_MECHANISM_ROOT.append(new MechanismLigament2d("PitcherLigament", PITCHER_LENGTH_METERS, 0, 10, new Color8Bit(Color.kGray)));
}
