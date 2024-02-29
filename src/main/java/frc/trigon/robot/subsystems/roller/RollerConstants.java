package frc.trigon.robot.subsystems.roller;

import frc.trigon.robot.utilities.SpeedMechanism2d;

public class RollerConstants {
    static final double
            NOTE_COLLECTION_CURRENT = 50,
            NOTE_COLLECTION_CURRENT_THRESHOLD_SECONDS = 0.14;
    static final double
            NOTE_COLLECTION_RUMBLE_DURATION_SECONDS = 0.6,
            NOTE_COLLECTION_RUMBLE_POWER = 1;


    public static final double GEAR_RATIO = 1.33333333333;
    private static final double MAXIMUM_DISPLAYABLE_VELOCITY = 12;
    static final SpeedMechanism2d ROLLER_MECHANISM = new SpeedMechanism2d("Mechanisms/RollerMechanism", MAXIMUM_DISPLAYABLE_VELOCITY);

    public enum RollerState {
        STOPPED(0),
        COLLECTING(3),
        AUTONOMOUS_FEEDING(2),
        FEEDING(3),
        SCORE_AMP(-6),
        SCORE_TRAP(-6),
        EJECTING(-3);

        final double voltage;

        RollerState(double voltage) {
            this.voltage = voltage;
        }
    }
}
