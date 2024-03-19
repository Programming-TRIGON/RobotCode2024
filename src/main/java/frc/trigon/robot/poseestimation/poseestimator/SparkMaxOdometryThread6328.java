// Copyright 2021-2024 FRC 6328
// http://github.com/Mechanical-Advantage
//
// This program is free software; you can redistribute it and/or
// modify it under the terms of the GNU General Public License
// version 3 as published by the Free Software Foundation or
// available in the root directory of this project.
//
// This program is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
// GNU General Public License for more details.

package frc.trigon.robot.poseestimation.poseestimator;

import edu.wpi.first.wpilibj.Notifier;
import edu.wpi.first.wpilibj.Timer;
import frc.trigon.robot.subsystems.swerve.Swerve;
import org.littletonrobotics.junction.Logger;

import java.util.ArrayList;
import java.util.List;
import java.util.Queue;
import java.util.concurrent.ArrayBlockingQueue;
import java.util.function.DoubleSupplier;

/**
 * Provides an interface for asynchronously reading high-frequency measurements to a set of queues.
 *
 * <p>This version is intended for devices like the SparkMax that require polling rather than a
 * blocking thread. A Notifier thread is used to gather samples with consistent timing.
 */
public class SparkMaxOdometryThread6328 extends Thread {
    private final List<DoubleSupplier> signals = new ArrayList<>();
    private final List<Queue<Double>> queues = new ArrayList<>();
    private final Queue<Double> timestamps = new ArrayBlockingQueue<>(100);
    private static SparkMaxOdometryThread6328 instance = null;

    public static SparkMaxOdometryThread6328 getInstance() {
        if (instance == null) {
            instance = new SparkMaxOdometryThread6328();
        }
        return instance;
    }

    private SparkMaxOdometryThread6328() {
        Notifier notifier = new Notifier(this::periodic);
        notifier.setName("SparkMaxOdometryThread");
        Timer.delay(1);
        notifier.startPeriodic(1.0 / PoseEstimatorConstants.ODOMETRY_FREQUENCY_HERTZ);
    }

    public Queue<Double> getTimestampQueue() {
        return timestamps;
    }

    public Queue<Double> registerSignal(DoubleSupplier signal) {
        Queue<Double> queue = new ArrayBlockingQueue<>(100);
        Swerve.ODOMETRY_LOCK.lock();
        try {
            signals.add(signal);
            queues.add(queue);
        } finally {
            Swerve.ODOMETRY_LOCK.unlock();
        }
        return queue;
    }

    private void periodic() {
        Swerve.ODOMETRY_LOCK.lock();
        timestamps.offer(Logger.getRealTimestamp() / 1.0e6);
        try {
            for (int i = 0; i < signals.size(); i++) {
                queues.get(i).offer(signals.get(i).getAsDouble());
            }
        } finally {
            Swerve.ODOMETRY_LOCK.unlock();
        }
    }
}