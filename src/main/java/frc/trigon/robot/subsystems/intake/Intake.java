package frc.trigon.robot.subsystems.intake;

import edu.wpi.first.math.geometry.*;
import edu.wpi.first.units.Measure;
import edu.wpi.first.units.Units;
import edu.wpi.first.units.Voltage;
import edu.wpi.first.wpilibj.sysid.SysIdRoutineLog;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.trigon.robot.RobotContainer;
import frc.trigon.robot.commands.Commands;
import frc.trigon.robot.constants.CommandConstants;
import frc.trigon.robot.subsystems.MotorSubsystem;
import org.littletonrobotics.junction.Logger;

public class Intake extends MotorSubsystem {
    private final IntakeInputsAutoLogged intakeInputs = new IntakeInputsAutoLogged();
    private final IntakeIO intakeIO = IntakeIO.generateIO();

    public Intake() {
        setName("Intake");
        Commands.getDelayedCommand(1, this::configureChangingDefaultCommand).schedule();
    }

    @Override
    public void periodic() {
        intakeIO.updateInputs(intakeInputs);
        Logger.processInputs("Intake", intakeInputs);
        updateMechanisms();
    }

    @Override
    public void drive(Measure<Voltage> voltageMeasure) {
        intakeIO.setTargetAngleMotorVoltage(voltageMeasure.in(Units.Volts));
    }

    @Override
    public void updateLog(SysIdRoutineLog log) {
        log.motor("IntakeAngle")
                .angularPosition(Units.Degrees.of(intakeInputs.anglePositionDegrees))
                .angularVelocity(Units.DegreesPerSecond.of(intakeInputs.angleVelocityDegreesPerSecond))
                .voltage(Units.Volts.of(intakeInputs.angleMotorVoltage));
    }

    @Override
    public SysIdRoutine.Config getSysIdConfig() {
        return IntakeConstants.SYSID_CONFIG;
    }

    @Override
    public void stop() {
        intakeIO.stop();
        IntakeConstants.COLLECTOR_MECHANISM.setTargetVelocity(0);
    }

    public boolean isOpenForElevator() {
        return IntakeConstants.OPEN_FOR_ELEVATOR_DEGREES > intakeInputs.anglePositionDegrees;
    }

    void setTargetState(Rotation2d angle, double collectionVoltage) {
        intakeIO.setTargetAngle(angle);
        intakeIO.setTargetCollectionVoltage(collectionVoltage);
        IntakeConstants.COLLECTOR_MECHANISM.setTargetVelocity(collectionVoltage);
    }

    private void updateMechanisms() {
        IntakeConstants.COLLECTOR_MECHANISM.updateMechanism(intakeInputs.collectionMotorVoltage);
        IntakeConstants.CURRENT_POSITION_INTAKE_LIGAMENT.setAngle(intakeInputs.anglePositionDegrees);
        IntakeConstants.TARGET_POSITION_INTAKE_LIGAMENT.setAngle(intakeInputs.angleMotorProfiledSetpointDegrees);
        Logger.recordOutput("Mechanisms/IntakeAngleMechanism", IntakeConstants.INTAKE_MECHANISM);
        Logger.recordOutput("Poses/Components/IntakePose", getIntakePose());
    }

    private Pose3d getIntakePose() {
        final Transform3d intakeTransform = new Transform3d(
                new Translation3d(),
                new Rotation3d(0, edu.wpi.first.math.util.Units.degreesToRadians(-intakeInputs.anglePositionDegrees), 0)
        );
        return IntakeConstants.INTAKE_ORIGIN_POINT.transformBy(intakeTransform);
    }

    private void configureChangingDefaultCommand() {
        final Trigger shouldOpenByDefaultTrigger = new Trigger(() -> (RobotContainer.ELEVATOR.isWithinHittingIntakeZone() || (RobotContainer.ELEVATOR.isClosing() && !RobotContainer.ELEVATOR.isClosed()) || CommandConstants.IS_CLIMBING));
        shouldOpenByDefaultTrigger.onFalse(new InstantCommand(this::defaultToResting));
        shouldOpenByDefaultTrigger.onTrue(new InstantCommand(this::defaultToOpening));
    }

    private void defaultToOpening() {
        changeDefaultCommand(IntakeCommands.getSetTargetStateCommand(IntakeConstants.IntakeState.OPENING));
    }

    private void defaultToResting() {
        changeDefaultCommand(IntakeCommands.getSetTargetStateCommand(IntakeConstants.IntakeState.RESTING));
    }
}