package frc.trigon.robot.subsystems.climber;

import edu.wpi.first.units.Units;
import edu.wpi.first.wpilibj.smartdashboard.Mechanism2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismLigament2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismRoot2d;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj.util.Color8Bit;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;

public class ClimberConstants {
    public static final double
            DRUM_RADIUS_METERS = 0.2,
            DIAMETER_METERS = DRUM_RADIUS_METERS * 2;
    public static final double RETRACTED_CLIMBER_LENGTH_METERS = 0.1;
    public static final double GEAR_RATIO = 10;
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
    private static final MechanismRoot2d MECHANISM_ROOT = MECHANISM.getRoot("MotorRoot", MECHANISM_ROOT_X, MECHANISM_ROOT_Y);
    static final MechanismLigament2d
            MECHANISM_CURRENT_POSITION_LIGAMENT = MECHANISM_ROOT.append(new MechanismLigament2d("ZMotorCurrentPositionLigament", 0, LIGAMENT_ANGLE, MECHANISM_LINE_WIDTH, new Color8Bit(Color.kBlue))),
            MECHANISM_TARGET_POSITION_LIGAMENT = MECHANISM_ROOT.append(new MechanismLigament2d("MotorTargetPositionLigament", 0, LIGAMENT_ANGLE, MECHANISM_LINE_WIDTH, new Color8Bit(Color.kGray)));

    static final SysIdRoutine.Config SYSID_CONFIG = new SysIdRoutine.Config(
            Units.Volts.of(1).per(Units.Second.of(1)),
            Units.Volts.of(7),
            null,
            null
    );

    public enum ClimberState {
        LOWERED(0.2),
        RAISED(0.6);

        final double positionMeters;

        ClimberState(double positionMeters) {
            this.positionMeters = positionMeters;
        }
    }
}
