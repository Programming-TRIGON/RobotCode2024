//
// Source code recreated from a .class file by IntelliJ IDEA
// (powered by FernFlower decompiler)
//

package frc.trigon.robot.commands;

import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Subsystem;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.networktables.LoggedDashboardBoolean;
import org.littletonrobotics.junction.networktables.LoggedDashboardNumber;

import java.util.Arrays;
import java.util.function.DoubleConsumer;
import java.util.function.DoubleSupplier;
import java.util.function.Supplier;

public class WheelRadiusCharacterizationCommand extends Command {
    private static final LoggedDashboardNumber CHARACTERIZATION_SPEED = new LoggedDashboardNumber("WheelRadiusCharacterization/SpeedRadiansPerSecond", 1.0);
    private static final LoggedDashboardNumber ROTATION_RATE_LIMIT = new LoggedDashboardNumber("WheelRadiusCharacterization/RotationRateLimit", 1.0);
    private static final LoggedDashboardBoolean SHOULD_MOVE_CLOCKWISE = new LoggedDashboardBoolean("WheelRadiusCharacterization/ShouldMoveClockwise", false);
    private final double[] wheelDistancesFromCenterMeters;
    private final Supplier<double[]> wheelPositionsRadiansSupplier;
    private final DoubleSupplier gyroYawRadiansSupplier;
    private final DoubleConsumer runWheelRadiusCharacterization;
    private SlewRateLimiter rotationSlewRateLimiter;
    private double startingGyroYawRadians;
    private double accumulatedGyroYawRadians;
    private double[] startingWheelPositions;
    private double driveWheelsRadius;

    public WheelRadiusCharacterizationCommand(Translation2d[] wheelDistancesFromCenterMeters, Supplier<double[]> wheelPositionsRadiansSupplier, DoubleSupplier gyroYawRadiansSupplier, DoubleConsumer runWheelRadiusCharacterization, SubsystemBase requirement) {
        this.rotationSlewRateLimiter = new SlewRateLimiter(ROTATION_RATE_LIMIT.get());
        this.wheelDistancesFromCenterMeters = Arrays.stream(wheelDistancesFromCenterMeters).mapToDouble(Translation2d::getNorm).toArray();
        this.wheelPositionsRadiansSupplier = wheelPositionsRadiansSupplier;
        this.gyroYawRadiansSupplier = gyroYawRadiansSupplier;
        this.runWheelRadiusCharacterization = runWheelRadiusCharacterization;
        this.addRequirements(new Subsystem[]{requirement});
    }

    public WheelRadiusCharacterizationCommand(double[] wheelDistancesFromCenterMeters, Supplier<double[]> wheelPositionsRadiansSupplier, DoubleSupplier gyroYawRadiansSupplier, DoubleConsumer runWheelRadiusCharacterization, SubsystemBase requirement) {
        this.rotationSlewRateLimiter = new SlewRateLimiter(ROTATION_RATE_LIMIT.get());
        this.wheelDistancesFromCenterMeters = wheelDistancesFromCenterMeters;
        this.wheelPositionsRadiansSupplier = wheelPositionsRadiansSupplier;
        this.gyroYawRadiansSupplier = gyroYawRadiansSupplier;
        this.runWheelRadiusCharacterization = runWheelRadiusCharacterization;
        this.addRequirements(new Subsystem[]{requirement});
    }

    public void initialize() {
        this.configureStartingStats();
    }

    public void execute() {
        this.driveMotors();
        this.accumulatedGyroYawRadians = this.getAccumulatedGyroYaw();
        this.driveWheelsRadius = this.calculateDriveWheelRadius();
        Logger.recordOutput("WheelRadiusCharacterization/AccumulatedGyroYawRadians", this.accumulatedGyroYawRadians);
        Logger.recordOutput("RadiusCharacterization/DriveWheelRadius", this.driveWheelsRadius);
    }

    public void end(boolean interrupted) {
        this.printResults();
    }

    private void configureStartingStats() {
        this.startingGyroYawRadians = this.gyroYawRadiansSupplier.getAsDouble();
        this.startingWheelPositions = (double[])this.wheelPositionsRadiansSupplier.get();
        this.accumulatedGyroYawRadians = 0.0;
        this.rotationSlewRateLimiter = new SlewRateLimiter(ROTATION_RATE_LIMIT.get());
        this.rotationSlewRateLimiter.reset(0.0);
    }

    private void driveMotors() {
        this.runWheelRadiusCharacterization.accept(this.rotationSlewRateLimiter.calculate((double)this.getRotationDirection() * CHARACTERIZATION_SPEED.get()));
    }

    private double getAccumulatedGyroYaw() {
        return Math.abs(this.startingGyroYawRadians - this.gyroYawRadiansSupplier.getAsDouble());
    }

    private double calculateDriveWheelRadius() {
        this.driveWheelsRadius = 0.0;
        double[] wheelPositionsRadians = (double[])this.wheelPositionsRadiansSupplier.get();

        for(int i = 0; i < 4; ++i) {
            final double accumulatedWheelRadians = Math.abs(wheelPositionsRadians[i] - this.startingWheelPositions[i]);
            final double currentWheelRadius = accumulatedGyroYawRadians * this.wheelDistancesFromCenterMeters[i] / accumulatedWheelRadians;
            this.driveWheelsRadius += currentWheelRadius;
            Logger.recordOutput("RadiusCharacterization/AccumulatedWheelRadians" + i, accumulatedWheelRadians);
            Logger.recordOutput("RadiusCharacterization/WheelRadius" + i, currentWheelRadius);
        }

        return this.driveWheelsRadius /= 4.0;
    }

    private void printResults() {
        if (this.accumulatedGyroYawRadians <= 6.283185307179586) {
            System.out.println("Not enough data for characterization");
        } else {
            System.out.println("Drive Wheel Radius: " + this.driveWheelsRadius + " meters");
        }

    }

    private int getRotationDirection() {
        return SHOULD_MOVE_CLOCKWISE.get() ? -1 : 1;
    }
}
