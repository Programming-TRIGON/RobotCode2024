//
// Source code recreated from a .class file by IntelliJ IDEA
// (powered by FernFlower decompiler)
//

package frc.trigon.robot.commands;

import edu.wpi.first.wpilibj2.command.*;
import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.networktables.LoggedDashboardBoolean;
import org.littletonrobotics.junction.networktables.LoggedDashboardNumber;

import java.util.function.DoubleConsumer;
import java.util.function.DoubleSupplier;

public class GearRatioCalculationCommand extends Command {
    private final DoubleSupplier rotorPositionSupplier;
    private final DoubleSupplier encoderPositionSupplier;
    private final DoubleConsumer runGearRatioCalculation;
    private final String subsystemName;
    private final LoggedDashboardNumber movementVoltage;
    private final LoggedDashboardBoolean shouldMoveClockwise;
    private double startingRotorPosition;
    private double startingEncoderPosition;
    private double gearRatio;

    public GearRatioCalculationCommand(DoubleSupplier rotorPositionSupplier, DoubleSupplier encoderPositionSupplier, DoubleConsumer runGearRatioCalculation, SubsystemBase requirement) {
        this.rotorPositionSupplier = rotorPositionSupplier;
        this.encoderPositionSupplier = encoderPositionSupplier;
        this.runGearRatioCalculation = runGearRatioCalculation;
        this.subsystemName = requirement.getName();
        this.movementVoltage = new LoggedDashboardNumber("GearRatioCalculation/" + this.subsystemName + "/Voltage", 1.0);
        this.shouldMoveClockwise = new LoggedDashboardBoolean("GearRatioCalculation/" + this.subsystemName + "/ShouldMoveClockwise", false);
        this.addRequirements(new Subsystem[]{requirement});
    }

    public void initialize() {
        new WaitCommand(0.5).andThen(new InstantCommand( () -> {
            this.startingRotorPosition = this.rotorPositionSupplier.getAsDouble();
            this.startingEncoderPosition = this.encoderPositionSupplier.getAsDouble();
        })).schedule();
        this.startingRotorPosition = this.rotorPositionSupplier.getAsDouble();
        this.startingEncoderPosition = this.encoderPositionSupplier.getAsDouble();
    }

    public void execute() {
        this.runGearRatioCalculation.accept(this.movementVoltage.get() * (double)this.getRotationDirection());
        this.gearRatio = this.calculateGearRatio();
        Logger.recordOutput("GearRatioCalculation/" + this.subsystemName + "/RotorDistance", this.getRotorDistance());
        Logger.recordOutput("GearRatioCalculation/" + this.subsystemName + "/EncoderDistance", this.getEncoderDistance());
        Logger.recordOutput("GearRatioCalculation/" + this.subsystemName + "/GearRatio", this.gearRatio);
    }

    public void end(boolean interrupted) {
        this.printResult();
    }

    private double calculateGearRatio() {
        double currentRotorPosition = this.getRotorDistance();
        double currentEncoderPosition = this.getEncoderDistance();
        return currentRotorPosition / currentEncoderPosition;
    }

    private double getRotorDistance() {
        return this.startingRotorPosition - this.rotorPositionSupplier.getAsDouble();
    }

    private double getEncoderDistance() {
        return this.startingEncoderPosition - this.encoderPositionSupplier.getAsDouble();
    }

    private int getRotationDirection() {
        return this.shouldMoveClockwise.get() ? -1 : 1;
    }

    private void printResult() {
        System.out.println(this.subsystemName + " Gear Ratio: " + this.gearRatio);
    }
}
