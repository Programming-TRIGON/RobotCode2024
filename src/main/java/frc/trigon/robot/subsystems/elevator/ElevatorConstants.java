package frc.trigon.robot.subsystems.elevator;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.units.Units;
import edu.wpi.first.wpilibj.smartdashboard.Mechanism2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismLigament2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismRoot2d;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj.util.Color8Bit;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;

public class ElevatorConstants {
    public static final double
            DRUM_RADIUS_METERS = 0.0222997564,
            DRUM_DIAMETER_METERS = DRUM_RADIUS_METERS * 2;
    public static final double RETRACTED_ELEVATOR_LENGTH_METERS = 0.630;
    public static final double GEAR_RATIO = 6.82;
    private static final double
            ELEVATOR_MECHANISM_WIDTH = 5,
            ELEVATOR_MECHANISM_HEIGHT = 5,
            ELEVATOR_MECHANISM_ROOT_X = 2,
            ELEVATOR_MECHANISM_ROOT_Y = 2,
            LIGAMENT_LINE_WIDTH = 10;
    static final Mechanism2d ELEVATOR_MECHANISM = new Mechanism2d(
            ELEVATOR_MECHANISM_WIDTH,
            ELEVATOR_MECHANISM_HEIGHT
    );

    private static final MechanismRoot2d ELEVATOR_ROOT = ELEVATOR_MECHANISM.getRoot("ElevatorRoot", ELEVATOR_MECHANISM_ROOT_X, ELEVATOR_MECHANISM_ROOT_Y);
    static final MechanismLigament2d
            ELEVATOR_LIGAMENT = ELEVATOR_ROOT.append(new MechanismLigament2d("ZElevatorLigament", 0, 0, LIGAMENT_LINE_WIDTH, new Color8Bit(Color.kBlue))),
            TARGET_ELEVATOR_POSITION_LIGAMENT = ELEVATOR_ROOT.append(new MechanismLigament2d("TargetElevatorPositionLigament", 0, 0, LIGAMENT_LINE_WIDTH, new Color8Bit(Color.kGray)));

    static final SysIdRoutine.Config SYSID_CONFIG = new SysIdRoutine.Config(
            Units.Volts.of(1).per(Units.Second.of(1)),
            Units.Volts.of(7),
            null,
            null
    );

    static final Pose3d
            ELEVATOR_ORIGIN_POINT = new Pose3d(0.10018, 0, 0.04, new Rotation3d(0, edu.wpi.first.math.util.Units.degreesToRadians(10), 0)),
            ROLLER_ORIGIN_POINT = new Pose3d(0.10018, 0, 0.06, new Rotation3d(0, edu.wpi.first.math.util.Units.degreesToRadians(10), 0));

    public enum ElevatorState {
        STOPPED(0),
        COLLECTION(0),
        FEEDING(0),
        SCORE_AMP(0),
        SCORE_TRAP(0);

        final double positionMeters;

        ElevatorState(double positionMeters) {
            this.positionMeters = positionMeters;
        }
    }
}