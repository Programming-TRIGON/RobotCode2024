package frc.trigon.robot.subsystems.elevator;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.units.Measure;
import edu.wpi.first.units.Units;
import edu.wpi.first.units.Voltage;
import edu.wpi.first.wpilibj.sysid.SysIdRoutineLog;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.trigon.robot.subsystems.MotorSubsystem;
import frc.trigon.robot.utilities.Conversions;
import org.littletonrobotics.junction.Logger;

public class Elevator extends MotorSubsystem {
    private final ElevatorIO elevatorIO = ElevatorIO.generateIO();
    private final ElevatorInputsAutoLogged elevatorInputs = new ElevatorInputsAutoLogged();
    private ElevatorConstants.ElevatorState targetState = ElevatorConstants.ElevatorState.RESTING;

    public Elevator() {
        setName("Elevator");
    }

    @Override
    public void periodic() {
        elevatorIO.updateInputs(elevatorInputs);
        Logger.processInputs("Elevator", elevatorInputs);
        updateNetworkTables();
    }

    @Override
    public void updateLog(SysIdRoutineLog log) {
        log.motor("Elevator")
                .linearPosition(Units.Meters.of(elevatorInputs.positionRevolutions))
                .linearVelocity(Units.MetersPerSecond.of(elevatorInputs.velocityRevolutionsPerSecond))
                .voltage(Units.Volts.of(elevatorInputs.motorVoltage));
    }

    @Override
    public SysIdRoutine.Config getSysIdConfig() {
        return ElevatorConstants.SYSID_CONFIG;
    }

    @Override
    public void setBrake(boolean brake) {
        elevatorIO.setBrake(brake);
    }

    @Override
    public void stop() {
        elevatorIO.stop();
    }

    @Override
    public void drive(Measure<Voltage> voltageMeasure) {
        elevatorIO.setTargetVoltage(voltageMeasure.in(Units.Volts));
    }

    public boolean atTargetState() {
        return Math.abs(this.targetState.positionMeters - getPositionMeters()) < ElevatorConstants.TOLERANCE_METERS;
    }

    public boolean isResting() {
        return targetState == ElevatorConstants.ElevatorState.RESTING;
    }

    void setTargetState(ElevatorConstants.ElevatorState targetState) {
        this.targetState = targetState;
        setTargetPosition(targetState.positionMeters, targetState.speedPercentage);
    }

    void setTargetPosition(double targetPositionMeters, double speedPercentage) {
        elevatorIO.setTargetPosition(toRevolutions(targetPositionMeters), speedPercentage);
    }

    private void updateNetworkTables() {
        updateMechanism();
        Logger.recordOutput("Elevator/ElevatorPositionMeters", getPositionMeters());
        Logger.recordOutput("Elevator/ElevatorVelocityMetersPerSecond", toMeters(elevatorInputs.velocityRevolutionsPerSecond));
    }

    private void updateMechanism() {
        ElevatorConstants.ELEVATOR_LIGAMENT.setLength(toMeters(elevatorInputs.positionRevolutions) + ElevatorConstants.RETRACTED_ELEVATOR_LENGTH_METERS);
        ElevatorConstants.TARGET_ELEVATOR_POSITION_LIGAMENT.setLength(toMeters(elevatorInputs.profiledSetpointRevolutions) + ElevatorConstants.RETRACTED_ELEVATOR_LENGTH_METERS);
        Logger.recordOutput("Poses/Components/ElevatorPose", getElevatorComponentPose());
        Logger.recordOutput("Poses/Components/TransporterPose", getTransporterComponentPose());
        Logger.recordOutput("Mechanisms/ElevatorMechanism", ElevatorConstants.ELEVATOR_MECHANISM);
    }

    private Pose3d getElevatorComponentPose() {
        final Transform3d elevatorTransform = new Transform3d(
                new Translation3d(0, 0, getPositionMeters()),
                new Rotation3d()
        );
        return ElevatorConstants.ELEVATOR_ORIGIN_POINT.transformBy(elevatorTransform);
    }

    public Pose3d getTransporterComponentPose() {
        final Transform3d transporterTransform = new Transform3d(
                new Translation3d(0, 0, getPositionMeters() * 2),
                new Rotation3d()
        );
        return ElevatorConstants.TRANSPORTER_ORIGIN_POINT.transformBy(transporterTransform);
    }

    private double getPositionMeters() {
        return toMeters(elevatorInputs.positionRevolutions);
    }

    private double toMeters(double revolutions) {
        return Conversions.revolutionsToDistance(revolutions, ElevatorConstants.DRUM_DIAMETER_METERS);
    }

    private double toRevolutions(double meters) {
        return Conversions.distanceToRevolutions(meters, ElevatorConstants.DRUM_DIAMETER_METERS);
    }
}