package frc.trigon.robot.commands;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.*;
import frc.trigon.robot.RobotContainer;
import frc.trigon.robot.constants.CommandConstants;
import frc.trigon.robot.constants.OperatorConstants;
import frc.trigon.robot.constants.ShootingConstants;
import frc.trigon.robot.subsystems.MotorSubsystem;
import frc.trigon.robot.subsystems.climber.ClimberCommands;
import frc.trigon.robot.subsystems.climber.ClimberConstants;
import frc.trigon.robot.subsystems.elevator.ElevatorCommands;
import frc.trigon.robot.subsystems.elevator.ElevatorConstants;
import frc.trigon.robot.subsystems.intake.IntakeCommands;
import frc.trigon.robot.subsystems.intake.IntakeConstants;
import frc.trigon.robot.subsystems.pitcher.PitcherCommands;
import frc.trigon.robot.subsystems.roller.RollerCommands;
import frc.trigon.robot.subsystems.roller.RollerConstants;
import frc.trigon.robot.subsystems.shooter.ShooterCommands;
import frc.trigon.robot.subsystems.swerve.SwerveCommands;
import frc.trigon.robot.utilities.ShootingCalculations;
import org.littletonrobotics.junction.Logger;

import java.util.function.BooleanSupplier;

public class Commands {
    private static final ShootingCalculations SHOOTING_CALCULATIONS = ShootingCalculations.getInstance();
    private static boolean IS_BRAKING = true;

    /**
     * @return a command that toggles between the swerve's default command, from field relative to self relative
     */
    public static Command getToggleFieldAndSelfRelativeDriveCommand() {
        return new InstantCommand(() -> {
            if (RobotContainer.SWERVE.getDefaultCommand().equals(CommandConstants.FIELD_RELATIVE_DRIVE_COMMAND))
                RobotContainer.SWERVE.setDefaultCommand(CommandConstants.SELF_RELATIVE_DRIVE_COMMAND);
            else
                RobotContainer.SWERVE.setDefaultCommand(CommandConstants.FIELD_RELATIVE_DRIVE_COMMAND);

            RobotContainer.SWERVE.getDefaultCommand().schedule();
        });
    }

    public static Command getToggleBrakeCommand() {
        return new InstantCommand(() -> {
            IS_BRAKING = !IS_BRAKING;
            MotorSubsystem.setAllSubsystemsBrakeAsync(IS_BRAKING);

            if (IS_BRAKING)
                CommandConstants.STATIC_WHITE_LED_COLOR_COMMAND.cancel();
            else
                CommandConstants.STATIC_WHITE_LED_COLOR_COMMAND.schedule();
        }).ignoringDisable(true);
    }

    public static Command getDelayedCommand(double delaySeconds, Runnable toRun) {
        return new WaitCommand(delaySeconds).andThen(toRun).ignoringDisable(true);
    }

    public static Command getScoreInAmpCommand() {
        return new ParallelCommandGroup(
                IntakeCommands.getSetTargetStateCommand(IntakeConstants.IntakeState.OPENING),
                runWhen(ElevatorCommands.getSetTargetStateCommand(ElevatorConstants.ElevatorState.SCORE_AMP), RobotContainer.INTAKE::isOpenForElevator),
                SwerveCommands.getClosedLoopFieldRelativeDriveCommand(
                        () -> CommandConstants.calculateDriveStickAxisValue(OperatorConstants.DRIVER_CONTROLLER.getLeftY()),
                        () -> CommandConstants.calculateDriveStickAxisValue(OperatorConstants.DRIVER_CONTROLLER.getLeftX()),
                        () -> Rotation2d.fromDegrees(90)
                ),
                runWhenContinueTriggerPressed(RollerCommands.getSetTargetStateCommand(RollerConstants.RollerState.SCORE_AMP))
        );
    }

    public static Command getShootAtSpeakerCommand() {
        return new ParallelCommandGroup(
                getPrepareShootingCommand(),
                runWhenContinueTriggerPressed(RollerCommands.getSetTargetStateCommand(RollerConstants.RollerState.FEEDING))
        );
    }

    public static Command getPrepareShootingCommand() {
        return new ParallelCommandGroup(
                getUpdateShootingCalculationsCommand(),
                PitcherCommands.getPitchToSpeakerCommand(),
                ShooterCommands.getShootAtSpeakerCommand(),
                ElevatorCommands.getSetTargetStateCommand(ElevatorConstants.ElevatorState.RESTING),
                SwerveCommands.getClosedLoopFieldRelativeDriveCommand(
                        () -> CommandConstants.calculateDriveStickAxisValue(OperatorConstants.DRIVER_CONTROLLER.getLeftY()),
                        () -> CommandConstants.calculateDriveStickAxisValue(OperatorConstants.DRIVER_CONTROLLER.getLeftX()),
                        SHOOTING_CALCULATIONS::calculateTargetRobotAngle
                )
        );
    }

    public static Command getPrepareShootingForAutoCommand() {
        return new ParallelCommandGroup(
                getUpdateShootingCalculationsCommand(),
                PitcherCommands.getPitchToSpeakerCommand(),
                ShooterCommands.getShootAtSpeakerCommand(),
                ElevatorCommands.getSetTargetStateCommand(ElevatorConstants.ElevatorState.RESTING)
        );
    }

    public static Command getCloseShotCommand() {
        return new SequentialCommandGroup(
                getPrepareForCloseShotCommand().until(() -> RobotContainer.SHOOTER.atTargetShootingVelocity() && RobotContainer.PITCHER.atTargetPitch()),
                RollerCommands.getSetTargetStateCommand(RollerConstants.RollerState.FEEDING).alongWith(getPrepareForCloseShotCommand())
        );
    }

    public static Command getPrepareForCloseShotCommand() {
        return new ParallelCommandGroup(
                ShooterCommands.getSetTargetShootingVelocityCommand(ShootingConstants.CLOSE_SHOT_VELOCITY_METERS_PER_SECOND),
                PitcherCommands.getSetTargetPitchCommand(ShootingConstants.CLOSE_SHOT_ANGLE),
                ElevatorCommands.getSetTargetStateCommand(ElevatorConstants.ElevatorState.RESTING)
        );
    }

    public static Command getClimbCommand() {
        return new SequentialCommandGroup(
                new InstantCommand(() -> {
                    CommandConstants.IS_CLIMBING = true;
                    Logger.recordOutput("IsClimbing", true);
                }),
                ClimberCommands.getSetTargetStateCommand(ClimberConstants.ClimberState.CLIMBING_PREPARATION).until(OperatorConstants.CONTINUE_TRIGGER),
                ClimberCommands.getSetTargetStateCommand(ClimberConstants.ClimberState.CLIMB).alongWith(
                        ElevatorCommands.getSetTargetStateCommand(ElevatorConstants.ElevatorState.SCORE_TRAP),
                        runWhen(RollerCommands.getSetTargetStateCommand(RollerConstants.RollerState.SCORE_TRAP), OperatorConstants.SECOND_CONTINUE_TRIGGER)
                )
        ).alongWith(IntakeCommands.getSetTargetStateCommand(IntakeConstants.IntakeState.OPENING));
    }

    public static Command getNoteCollectionCommand() {
        return new ParallelCommandGroup(
                getContinuousConditionalCommand(new AlignToNoteCommand(), duplicate(CommandConstants.FIELD_RELATIVE_DRIVE_COMMAND), () -> CommandConstants.SHOULD_ALIGN_TO_NOTE),
                getNonAssitedNoteCollectionCommand()
        );
    }

    public static Command getNonAssitedNoteCollectionCommand() {
        return new ParallelCommandGroup(
                runWhen(ElevatorCommands.getSetTargetStateCommand(ElevatorConstants.ElevatorState.RESTING), RobotContainer.INTAKE::isOpenForElevator),
                IntakeCommands.getSetTargetStateCommand(IntakeConstants.IntakeState.COLLECTING),
                RollerCommands.getSetTargetStateCommand(RollerConstants.RollerState.COLLECTING)
        );
    }

    public static Command getContinuousConditionalCommand(Command onTrue, Command onFalse, BooleanSupplier condition) {
        return new ConditionalCommand(
                onTrue.onlyWhile(condition),
                onFalse.onlyWhile(() -> !condition.getAsBoolean()),
                condition
        ).repeatedly();
    }

    public static Command runWhen(Command command, BooleanSupplier condition) {
        return command.onlyIf(condition).repeatedly();
    }

    public static Command duplicate(Command command) {
        return new FunctionalCommand(
                command::initialize,
                command::execute,
                command::end,
                command::isFinished,
                command.getRequirements().toArray(Subsystem[]::new)
        );
    }

    private static Command runWhenContinueTriggerPressed(Command command) {
        return runWhen(command, OperatorConstants.CONTINUE_TRIGGER);
    }

    private static Command getUpdateShootingCalculationsCommand() {
        return new RunCommand(SHOOTING_CALCULATIONS::updateCalculations);
    }
}
