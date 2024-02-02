package frc.trigon.robot.subsystems.elevator;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.units.Measure;
import edu.wpi.first.units.Units;
import edu.wpi.first.units.Voltage;
import edu.wpi.first.wpilibj.sysid.SysIdRoutineLog;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.trigon.robot.RobotContainer;
import frc.trigon.robot.commands.Commands;
import frc.trigon.robot.constants.OperatorConstants;
import frc.trigon.robot.subsystems.MotorSubsystem;
import frc.trigon.robot.utilities.Conversions;
import org.littletonrobotics.junction.Logger;

public class Elevator extends MotorSubsystem {
    private final ElevatorIO elevatorIO = ElevatorIO.generateIO();
    private final ElevatorInputsAutoLogged elevatorInputs = new ElevatorInputsAutoLogged();
    private ElevatorConstants.ElevatorState targetState = ElevatorConstants.ElevatorState.RESTING;

    public Elevator() {
        setName("Elevator");
        Commands.getDelayedCommand(1, this::configureChangingDefaultCommand).schedule();
    }

    @Override
    public void periodic() {
        elevatorIO.updateInputs(elevatorInputs);
        Logger.processInputs("Elevator", elevatorInputs);
        updateMechanism();
    }

    @Override
    public void updateLog(SysIdRoutineLog log) {
        log.motor("Elevator")
                .linearPosition(Units.Meters.of(Conversions.distanceToRevolutions(elevatorInputs.positionMeters, ElevatorConstants.DRUM_DIAMETER_METERS)))
                .linearVelocity(Units.MetersPerSecond.of(Conversions.distanceToRevolutions(elevatorInputs.velocityMetersPerSecond, ElevatorConstants.DRUM_DIAMETER_METERS)))
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
        return Math.abs(this.targetState.positionMeters - elevatorInputs.positionMeters) < ElevatorConstants.TOLERANCE_METERS;
    }

    public boolean isClosed() {
        return elevatorInputs.positionMeters < ElevatorConstants.OPEN_THRESHOLD_METERS;
    }

    void setTargetState(ElevatorConstants.ElevatorState targetState) {
        this.targetState = targetState;
        elevatorIO.setTargetPosition(targetState.positionMeters);

        if (shouldRumble(targetState))
            OperatorConstants.DRIVER_CONTROLLER.rumble(ElevatorConstants.OPENING_RUMBLE_DURATION_SECONDS, ElevatorConstants.OPENING_RUMBLE_POWER);
    }

    void setTargetPosition(double targetPositionMeters) {
        elevatorIO.setTargetPosition(targetPositionMeters);
    }

    void stayInPlace() {
        elevatorIO.setTargetPosition(elevatorInputs.positionMeters);
    }

    private boolean shouldRumble(ElevatorConstants.ElevatorState targetState) {
        return targetState != ElevatorConstants.ElevatorState.RESTING;
    }

    private void updateMechanism() {
        ElevatorConstants.ELEVATOR_LIGAMENT.setLength(elevatorInputs.positionMeters + ElevatorConstants.RETRACTED_ELEVATOR_LENGTH_METERS);
        ElevatorConstants.TARGET_ELEVATOR_POSITION_LIGAMENT.setLength(elevatorInputs.profiledSetpointMeters + ElevatorConstants.RETRACTED_ELEVATOR_LENGTH_METERS);
        Logger.recordOutput("Poses/Components/ElevatorPose", getElevatorComponentPose());
        Logger.recordOutput("Poses/Components/RollerPose", getRollerComponentPose());
        Logger.recordOutput("Mechanisms/ElevatorMechanism", ElevatorConstants.ELEVATOR_MECHANISM);
    }

    private Pose3d getElevatorComponentPose() {
        final Transform3d elevatorTransform = new Transform3d(
                new Translation3d(0, 0, elevatorInputs.positionMeters),
                new Rotation3d()
        );
        return ElevatorConstants.ELEVATOR_ORIGIN_POINT.transformBy(elevatorTransform);
    }

    public Pose3d getRollerComponentPose() {
        final Transform3d rollerTransform = new Transform3d(
                new Translation3d(0, 0, elevatorInputs.positionMeters * 2),
                new Rotation3d()
        );
        return ElevatorConstants.ROLLER_ORIGIN_POINT.transformBy(rollerTransform);
    }

    private void configureChangingDefaultCommand() {
        final Trigger shouldRestByDefaultTrigger = new Trigger(() -> RobotContainer.INTAKE.isOpenForElevator() || isClosed());
        shouldRestByDefaultTrigger.onTrue(new InstantCommand(this::defaultToResting));
        shouldRestByDefaultTrigger.onFalse(new InstantCommand(this::defaultToStayingInPlace));
    }

    private void defaultToStayingInPlace() {
        changeDefaultCommand(ElevatorCommands.getStayInPlaceCommand());
    }

    private void defaultToResting() {
        changeDefaultCommand(ElevatorCommands.getSetTargetStateCommand(ElevatorConstants.ElevatorState.RESTING));
    }
}