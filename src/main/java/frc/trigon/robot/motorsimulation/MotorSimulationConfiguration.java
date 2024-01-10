package frc.trigon.robot.motorsimulation;

public class MotorSimulationConfiguration {
    /**
     * The unit of conversion in which every value will be returned / calculated in.
     * This should be the same unit as the default "ticks" unit of your motor, to match the real world gains as closely as possible.
     */
    public double conversionFactor = 1;
    public double voltageCompensationSaturation = 12;
    public PIDConfigs pidConfigs = new PIDConfigs();
    public FeedForwardConfigs feedForwardConfigs = new FeedForwardConfigs();
    public MotionMagicConfigs motionMagicConfigs = new MotionMagicConfigs();

    public static class PIDConfigs {
        public double
                kP = 0,
                kI = 0,
                kD = 0;
        public boolean enableContinuousInput = false;
    }

    public static class FeedForwardConfigs {
        public double
                kS = 0,
                kG = 0,
                kV = 0,
                kA = 0;
    }

    public static class MotionMagicConfigs {
        public double
                maxVelocity = 0,
                maxAcceleration = 0;
    }
}
