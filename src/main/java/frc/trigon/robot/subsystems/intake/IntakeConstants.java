package frc.trigon.robot.subsystems.intake;

public class IntakeConstants {
    public static final double GEAR_RATIO = 1.66666666667;
    private static final double MAX_DISPLAYABLE_VELOCITY = 10;
    static final SpeedMechanism2d COLLECTOR_MECHANISM = new SpeedMechanism2d("Mechanisms/IntakeMechanism", MAX_DISPLAYABLE_VELOCITY);

    static final double
            NOTE_COLLECTION_CURRENT = 34,
            NOTE_COLLECTION_TIME_THRESHOLD_SECONDS = 0.15;

    public enum IntakeState {
        STOPPED(0),
        COLLECTING(10),
        EJECTING(-4);

        final double voltage;

        IntakeState(double voltage) {
            this.voltage = voltage;
        }
    }
}
