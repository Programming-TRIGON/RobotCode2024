package frc.trigon.robot.subsystems.roller;

import frc.trigon.robot.utilities.SpeedMechanism2d;

public class RollerConstants {
    static final double
            NOTE_COLLECTION_RUMBLE_DURATION_SECONDS = 0.6,
            NOTE_COLLECTION_RUMBLE_POWER = 1;

    public static final double GEAR_RATIO = 1.33333333333;
    private static final double MAXIMUM_DISPLAYABLE_VELOCITY = 100;
    static final SpeedMechanism2d ROLLER_MECHANISM = new SpeedMechanism2d("Mechanisms/RollerMechanism", MAXIMUM_DISPLAYABLE_VELOCITY);

    public enum RollerState {
        STOPPED(0),
        COLLECTING(-30),
        FEEDING(-40),
        SCORE_AMP(30),
        SCORE_TRAP(30);

        final double velocityRevolutionsPerSecond;

        RollerState(double velocityRevolutionsPerSecond) {
            this.velocityRevolutionsPerSecond = velocityRevolutionsPerSecond;
        }
    }
}
