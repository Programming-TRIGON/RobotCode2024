package frc.trigon.robot.subsystems.roller;

import frc.trigon.robot.utilities.SpeedMechanism2d;

public class RollerConstants {
    public static final double GEAR_RATIO = 1;
    private static final double MAX_DISPLAYABLE_VELOCITY = 1;
    static final SpeedMechanism2d ROLLER_MECHANISM = new SpeedMechanism2d("Roller", MAX_DISPLAYABLE_VELOCITY);

    public enum RollerState {
        STOPPED(0),
        COLLECTION(0),
        FEEDING(0),
        SCORE_AMP(0),
        SCORE_TRAP(0);

        final double velocityRevolutionsPerSecond;

        RollerState(double velocityRevolutionsPerSecond) {
            this.velocityRevolutionsPerSecond = velocityRevolutionsPerSecond;
        }
    }
}
