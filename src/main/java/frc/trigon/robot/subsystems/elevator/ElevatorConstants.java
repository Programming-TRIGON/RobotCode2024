package frc.trigon.robot.subsystems.elevator;

import edu.wpi.first.wpilibj.smartdashboard.Mechanism2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismLigament2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismRoot2d;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj.util.Color8Bit;

public class ElevatorConstants {
    public static final double RETRACTED_ARM_LENGTH_METERS = 0.05;
    public static final double GEAR_RATIO = 1;
    private static final double
            ELEVATOR_MECHANISM_WIDTH = 5,
            ELEVATOR_MECHANISM_HEIGHT = 5,
            ELEVATOR_MECHANISM_ROOT_X = 2,
            ELEVATOR_MECHANISM_ROOT_Y = 2,
            LIGAMENT_LINE_WIDTH = 10;
    private static final Mechanism2d ELEVATOR_MECHANISM = new Mechanism2d(
            ELEVATOR_MECHANISM_WIDTH,
            ELEVATOR_MECHANISM_HEIGHT
    );

    private static final MechanismRoot2d
            ELEVATOR_ROOT = ELEVATOR_MECHANISM.getRoot("ZElevatorRoot", ELEVATOR_MECHANISM_ROOT_X, ELEVATOR_MECHANISM_ROOT_Y),
            TARGET_ELEVATOR_POSITION_ROOT = ELEVATOR_MECHANISM.getRoot("TargetElevatorPositionRoot", ELEVATOR_MECHANISM_ROOT_X, ELEVATOR_MECHANISM_ROOT_Y);
    static final MechanismLigament2d
            ELEVATOR_LIGAMENT = ELEVATOR_ROOT.append(new MechanismLigament2d("ElevatorLigament", 0, 0, LIGAMENT_LINE_WIDTH, new Color8Bit(Color.kBlue))),
            TARGET_ELEVATOR_POSITION_LIGAMENT = TARGET_ELEVATOR_POSITION_ROOT.append(new MechanismLigament2d("TargetElevatorPositionLigament", 0, 0, LIGAMENT_LINE_WIDTH, new Color8Bit(Color.kGray)));

    public enum ElevatorState {
        BOTTOM(0),
        TOP(1);

        final double positionMeters;

        ElevatorState(double positionMeters) {
            this.positionMeters = positionMeters;
        }
    }
}