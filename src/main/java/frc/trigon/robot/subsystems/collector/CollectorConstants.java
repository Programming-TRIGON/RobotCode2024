package frc.trigon.robot.subsystems.collector;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.units.Units;
import edu.wpi.first.wpilibj.smartdashboard.Mechanism2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismLigament2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismRoot2d;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj.util.Color8Bit;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.trigon.robot.utilities.SpeedMechanism2d;

public class CollectorConstants {
    private static final double
            COLLECTOR_MECHANISM_WIDTH = 10,
            COLLECTOR_MECHANISM_HEIGHT = 10;
    private static final double
            MECHANISM_ROOT_X = 5,
            MECHANISM_ROOT_Y = 5;
    private static final double MECHANISM_LINE_WIDTH = 5;
    private static final double
            LIGAMENT_LENGTH = 1,
            LIGAMENT_ANGLE = 90;
    private static final double SPEED_MECHANISM_MAX_DISPLAYABLE_VELOCITY = 200;
    static final Mechanism2d COLLECTOR_MECHANISM = new Mechanism2d(COLLECTOR_MECHANISM_WIDTH, COLLECTOR_MECHANISM_HEIGHT);
    private static final MechanismRoot2d COLLECTOR_ROOT = COLLECTOR_MECHANISM.getRoot("ZCollectorRoot", MECHANISM_ROOT_X, MECHANISM_ROOT_Y);
    static final MechanismLigament2d MAIN_COLLECTOR_LIGAMENT = COLLECTOR_ROOT.append(new MechanismLigament2d("MainCollectorLigament", LIGAMENT_LENGTH, LIGAMENT_ANGLE, MECHANISM_LINE_WIDTH, new Color8Bit(Color.kBlue)));
    private static final MechanismLigament2d
            SECONDARY_COLLECTOR_LIGAMENT = MAIN_COLLECTOR_LIGAMENT.append(new MechanismLigament2d("SecondaryCollectorLigament", LIGAMENT_LENGTH, LIGAMENT_ANGLE, MECHANISM_LINE_WIDTH, new Color8Bit(Color.kBlue))),
            THIRD_COLLECTOR_LIGAMENT = SECONDARY_COLLECTOR_LIGAMENT.append(new MechanismLigament2d("ThirdCollectorLigament", LIGAMENT_LENGTH, LIGAMENT_ANGLE, MECHANISM_LINE_WIDTH, new Color8Bit(Color.kBlue)));
    static final SpeedMechanism2d SPEED_MECHANISM = new SpeedMechanism2d("CollectorSpeed", SPEED_MECHANISM_MAX_DISPLAYABLE_VELOCITY);
    static final SysIdRoutine.Config SYSID_CONFIG = new SysIdRoutine.Config(
            Units.Volts.of(1).per(Units.Second.of(1)),
            Units.Volts.of(7),
            null,
            null
    );

    public enum CollectorState {
        RESTING(0, Rotation2d.fromDegrees(90)),
        COLLECTING(-5, Rotation2d.fromDegrees(0)),
        OPENING(0, Rotation2d.fromDegrees(0));

        final double voltage;
        final Rotation2d angle;

        CollectorState(double voltage, Rotation2d angle) {
            this.voltage = voltage;
            this.angle = angle;
        }
    }
}
