package frc.trigon.robot.subsystems.collector;

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
import frc.trigon.robot.utilities.SpeedMechanism2d;

public class CollectorConstants {
    public static final double
            ANGLE_MOTOR_GEAR_RATIO = 70,
            COLLECTION_MOTOR_GEAR_RATIO = 1.66666666667;
    private static final double
            COLLECTOR_MECHANISM_WIDTH = 10,
            COLLECTOR_MECHANISM_HEIGHT = 10;
    private static final double
            MECHANISM_ROOT_X = 5,
            MECHANISM_ROOT_Y = 5;
    private static final double MECHANISM_LINE_WIDTH = 5;
    private static final double
            LIGAMENT_LENGTH = 1,
            LIGAMENT_ANGLE = -90;
    private static final double SPEED_MECHANISM_MAX_DISPLAYABLE_VELOCITY = 10;
    static final Mechanism2d COLLECTOR_MECHANISM = new Mechanism2d(COLLECTOR_MECHANISM_WIDTH, COLLECTOR_MECHANISM_HEIGHT);
    private static final MechanismRoot2d COLLECTOR_MECHANISM_ROOT = COLLECTOR_MECHANISM.getRoot("ZCollectorRoot", MECHANISM_ROOT_X, MECHANISM_ROOT_Y);
    static final MechanismLigament2d
            CURRENT_POSITION_COLLECTOR_LIGAMENT = COLLECTOR_MECHANISM_ROOT.append(new MechanismLigament2d("ZMainCollectorLigament", LIGAMENT_LENGTH, 0, MECHANISM_LINE_WIDTH, new Color8Bit(Color.kBlue))),
            TARGET_POSITION_COLLECTOR_LIGAMENT = COLLECTOR_MECHANISM_ROOT.append(new MechanismLigament2d("MainTargetPositionLigament", LIGAMENT_LENGTH, 0, MECHANISM_LINE_WIDTH, new Color8Bit(Color.kGray)));
    static final SpeedMechanism2d SPEED_MECHANISM = new SpeedMechanism2d("Mechanisms/CollectorSpeedMechanism", SPEED_MECHANISM_MAX_DISPLAYABLE_VELOCITY);

    static final Pose3d COLLECTOR_ORIGIN_POINT = new Pose3d(0.35198, 0, 0.14627, new Rotation3d(0, edu.wpi.first.math.util.Units.degreesToRadians(-15), 0));
    static final SysIdRoutine.Config SYSID_CONFIG = new SysIdRoutine.Config(
            Units.Volts.of(0.1).per(Units.Second.of(1)),
            Units.Volts.of(6),
            null
    );

    static {
        CURRENT_POSITION_COLLECTOR_LIGAMENT
                .append(new MechanismLigament2d("ZSecondaryCollectorLigament", LIGAMENT_LENGTH, LIGAMENT_ANGLE, MECHANISM_LINE_WIDTH, new Color8Bit(Color.kBlue)))
                .append(new MechanismLigament2d("ZThirdCollectorLigament", LIGAMENT_LENGTH, LIGAMENT_ANGLE, MECHANISM_LINE_WIDTH, new Color8Bit(Color.kBlue)));

        TARGET_POSITION_COLLECTOR_LIGAMENT
                .append(new MechanismLigament2d("SecondaryTargetPositionLigament", LIGAMENT_LENGTH, LIGAMENT_ANGLE, MECHANISM_LINE_WIDTH, new Color8Bit(Color.kGray)))
                .append(new MechanismLigament2d("ThirdTargetPositionLigament", LIGAMENT_LENGTH, LIGAMENT_ANGLE, MECHANISM_LINE_WIDTH, new Color8Bit(Color.kGray)));
    }

    public enum CollectorState {
        RESTING(0, Rotation2d.fromDegrees(90)),
        COLLECTING(-1, Rotation2d.fromDegrees(0)),
        OPENING(0, Rotation2d.fromDegrees(0));

        final double collectionVoltage;
        final Rotation2d angle;

        CollectorState(double collectionVoltage, Rotation2d angle) {
            this.collectionVoltage = collectionVoltage;
            this.angle = angle;
        }
    }
}
