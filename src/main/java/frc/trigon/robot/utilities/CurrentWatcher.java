package frc.trigon.robot.utilities;

import edu.wpi.first.wpilibj.Timer;

import java.util.ArrayList;
import java.util.List;
import java.util.function.DoubleSupplier;

/**
 * A class that checks if the motor's current passed a certain current limit for a certain amount of time. If so, runs a runnable.
 */
public class CurrentWatcher {
    private static final List<CurrentWatcher> REGISTERED_WATCHERS = new ArrayList<>();
    private final DoubleSupplier currentSupplier;
    private final double maxCurrent;
    private final double timeThreshold;
    private final Runnable callback;
    private double lastTimeBelowMaxCurrent;

    /**
     * Constructs a new CurrentWatcher that checks the current every 0.02 seconds, and if the current passes the current limit runs a runnable.
     *
     * @param currentSupplier      a supplier for the motor's current
     * @param maxCurrent           the current limit
     * @param timeThresholdSeconds the time needed for the current to exceed its limit in order to run the runnable, in seconds
     * @param callback             the runnable to run if the current passes it's limit
     */
    public CurrentWatcher(DoubleSupplier currentSupplier, double maxCurrent, double timeThresholdSeconds, Runnable callback) {
        this.currentSupplier = currentSupplier;
        this.maxCurrent = maxCurrent;
        this.timeThreshold = timeThresholdSeconds;
        this.callback = callback;

        REGISTERED_WATCHERS.add(this);
    }

    public static void checkCurrentForRegisteredWatchers() {
        for (CurrentWatcher currentWatcher : REGISTERED_WATCHERS)
            currentWatcher.checkCurrent();
    }

    private void checkCurrent() {
        if (isBelowMaxCurrent()) {
            lastTimeBelowMaxCurrent = -1;
            return;
        }

        if (isFirstPress())
            lastTimeBelowMaxCurrent = Timer.getFPGATimestamp();
        if (didPassTimeThreshold())
            callback.run();
    }

    private boolean isBelowMaxCurrent() {
        return currentSupplier.getAsDouble() < maxCurrent;
    }

    private boolean didPassTimeThreshold() {
        return getTimeAboveThreshold() >= timeThreshold;
    }

    private double getTimeAboveThreshold() {
        return Timer.getFPGATimestamp() - lastTimeBelowMaxCurrent;
    }

    private boolean isFirstPress() {
        return lastTimeBelowMaxCurrent == -1;
    }
}
