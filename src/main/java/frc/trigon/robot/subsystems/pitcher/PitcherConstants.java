package frc.trigon.robot.subsystems.pitcher;

import edu.wpi.first.units.Units;
import edu.wpi.first.wpilibj.smartdashboard.Mechanism2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismLigament2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismRoot2d;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj.util.Color8Bit;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.trigon.robot.constants.ShootingConstants;
import frc.trigon.robot.utilities.LinearInterpolation;

import java.util.ArrayList;

public class PitcherConstants {
    public static final double PITCHER_LENGTH_METERS = 0.5;

    static final double PITCH_TOLERANCE_DEGREES = 1;
    static final LinearInterpolation PITCH_INTERPOLATION = generateInterpolation();
    static final SysIdRoutine.Config SYS_ID_CONFIG = new SysIdRoutine.Config(
            Units.Volts.of(1).per(Units.Second),
            Units.Volts.of(7),
            null
    );

    static final Mechanism2d PITCHER_MECHANISM = new Mechanism2d(PITCHER_LENGTH_METERS * 2, PITCHER_LENGTH_METERS * 2, new Color8Bit(Color.kBlack));
    private static final MechanismRoot2d PITCHER_MECHANISM_ROOT = PITCHER_MECHANISM.getRoot("PitcherRoot", PITCHER_LENGTH_METERS, PITCHER_LENGTH_METERS);
    static final MechanismLigament2d
            PITCHER_LIGAMENT = PITCHER_MECHANISM_ROOT.append(new MechanismLigament2d("ZPitcherLigament", PITCHER_LENGTH_METERS, 0, 10, new Color8Bit(Color.kBlue))),
            TARGET_PITCHER_LIGAMENT = PITCHER_MECHANISM_ROOT.append(new MechanismLigament2d("PitcherLigament", PITCHER_LENGTH_METERS, 0, 10, new Color8Bit(Color.kGray)));

    private static LinearInterpolation generateInterpolation() {
        final ArrayList<LinearInterpolation.Point> points = new ArrayList<>();
        for (ShootingConstants.ShootingPosition position : ShootingConstants.SHOOTING_POSITIONS)
            points.add(new LinearInterpolation.Point(position.distanceMeters(), position.pitch().getRotations()));
        return new LinearInterpolation(points);
    }
}
