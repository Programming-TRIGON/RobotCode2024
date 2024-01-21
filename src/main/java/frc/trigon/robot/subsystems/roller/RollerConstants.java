package frc.trigon.robot.subsystems.roller;

import frc.trigon.robot.utilities.SpeedMechanism2d;

public class RollerConstants {
    public static final double GEAR_RATIO = 1;
    private static final double MAXIMUM_DISPLAYABLE_VELOCITY = 100;
    static final SpeedMechanism2d ROLLER_MECHANISM = new SpeedMechanism2d("Mechanisms/RollerMechanism", MAXIMUM_DISPLAYABLE_VELOCITY);

    public enum RollerState {
        STOPPED(0),
        COLLECTION(-30),
        FEEDING(-40),
        SCORE_AMP(30),
        SCORE_TRAP(30);

        final double velocityRevolutionsPerSecond;

        RollerState(double velocityRevolutionsPerSecond) {
            this.velocityRevolutionsPerSecond = velocityRevolutionsPerSecond;
        }
    }
}