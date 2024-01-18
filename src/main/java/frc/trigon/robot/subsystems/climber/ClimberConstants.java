package frc.trigon.robot.subsystems.climber;

import edu.wpi.first.units.Units;
import edu.wpi.first.wpilibj.smartdashboard.Mechanism2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismLigament2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismRoot2d;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj.util.Color8Bit;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;

public class ClimberConstants {
    public static final double DRUM_RADIUS = 1;

    private static final double
            MECHANISM_WIDTH = 1,
            MECHANISM_HEIGHT = 1;
    private static final double
            RIGHT_MECHANISM_ROOT_X = 0.75,
            RIGHT_MECHANISM_ROOT_Y = 0,
            LEFT_MECHANISM_ROOT_X = 0.25,
            LEFT_MECHANISM_ROOT_Y = 0;
    private static final double
            LIGAMENT_LENGTH = 0,
            LIGAMENT_ANGLE = 90;
    private static final double MECHANISM_LINE_WIDTH = 10;
    static final Mechanism2d MECHANISM = new Mechanism2d(MECHANISM_WIDTH, MECHANISM_HEIGHT);
    private static final MechanismRoot2d
            RIGHT_MECHANISM_ROOT = MECHANISM.getRoot("RightMotorRoot", RIGHT_MECHANISM_ROOT_X, RIGHT_MECHANISM_ROOT_Y),
            LEFT_MECHANISM_ROOT = MECHANISM.getRoot("LeftMotorRoot", LEFT_MECHANISM_ROOT_X, LEFT_MECHANISM_ROOT_Y);
    static final MechanismLigament2d
            RIGHT_MECHANISM_CURRENT_POSITION_LIGAMENT = RIGHT_MECHANISM_ROOT.append(new MechanismLigament2d("RightMotorCurrentPositionLigament", LIGAMENT_LENGTH, LIGAMENT_ANGLE, MECHANISM_LINE_WIDTH, new Color8Bit(Color.kBlue))),
            RIGHT_MECHANISM_TARGET_POSITION_LIGAMENT = RIGHT_MECHANISM_ROOT.append(new MechanismLigament2d("RightMotorTargetPositionLigament", LIGAMENT_LENGTH, LIGAMENT_ANGLE, MECHANISM_LINE_WIDTH, new Color8Bit(Color.kGray))),
            LEFT_MECHANISM_CURRENT_POSITION_LIGAMENT = LEFT_MECHANISM_ROOT.append(new MechanismLigament2d("LeftMotorCurrentPositionLigament", LIGAMENT_LENGTH, LIGAMENT_ANGLE, MECHANISM_LINE_WIDTH, new Color8Bit(Color.kBlue))),
            LEFT_MECHANISM_TARGET_POSITION_LIGAMENT = LEFT_MECHANISM_ROOT.append(new MechanismLigament2d("LeftMotorTargetPositionLigament", LIGAMENT_LENGTH, LIGAMENT_ANGLE, MECHANISM_LINE_WIDTH, new Color8Bit(Color.kGray)));

    static final SysIdRoutine.Config SYSID_CONFIG = new SysIdRoutine.Config(
            Units.Volts.of(1).per(Units.Second.of(1)),
            Units.Volts.of(7),
            null,
            null
    );

    public enum ClimberState {
        LOWERED(0.1),
        RAISED(0.5);

        final double averagePositionMeters;

        ClimberState(double averagePositionMeters) {
            this.averagePositionMeters = averagePositionMeters;
        }
    }
}
