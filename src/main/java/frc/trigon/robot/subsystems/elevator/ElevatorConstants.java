package frc.trigon.robot.subsystems.elevator;

import edu.wpi.first.units.Units;
import edu.wpi.first.wpilibj.smartdashboard.Mechanism2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismLigament2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismRoot2d;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj.util.Color8Bit;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;

public class ElevatorConstants {
    public static final double
            DRUM_RADIUS_METERS = 0.02,
            DRUM_DIAMETER_METERS = DRUM_RADIUS_METERS * 2;
    public static final double RETRACTED_ELEVATOR_LENGTH_METERS = 0.05;
    public static final double GEAR_RATIO = 1;
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