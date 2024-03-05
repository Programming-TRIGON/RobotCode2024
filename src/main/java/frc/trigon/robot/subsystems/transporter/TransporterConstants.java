package frc.trigon.robot.subsystems.transporter;

import frc.trigon.robot.utilities.SpeedMechanism2d;

public class TransporterConstants {
    static final double
            NOTE_COLLECTION_CURRENT = 28,
            NOTE_COLLECTION_CURRENT_THRESHOLD_SECONDS = 0.38;
    static final double
            NOTE_COLLECTION_RUMBLE_DURATION_SECONDS = 0.6,
            NOTE_COLLECTION_RUMBLE_POWER = 1;


    public static final double GEAR_RATIO = 1.33333333333;
    private static final double MAXIMUM_DISPLAYABLE_VELOCITY = 12;
    static final SpeedMechanism2d TRANSPORTER_MECHANISM = new SpeedMechanism2d("Mechanisms/TransporterMechanism", MAXIMUM_DISPLAYABLE_VELOCITY);

    public enum TransporterState {
        STOPPED(0),
        COLLECTING(4),
        AUTONOMOUS_FEEDING(12),
        FEEDING(12),
        SCORE_AMP(-12),
        ALIGNING_FOR_AMP(-5),
        ALIGNING_FOR_TRAP(3),
        SCORE_TRAP(-5),
        EJECTING(-4);

        final double voltage;

        TransporterState(double voltage) {
            this.voltage = voltage;
        }
    }
}
