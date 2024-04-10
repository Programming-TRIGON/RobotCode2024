package frc.trigon.robot.commands;

import com.pathplanner.lib.commands.PathPlannerAuto;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.*;
import frc.trigon.robot.RobotContainer;
import frc.trigon.robot.constants.*;
import frc.trigon.robot.subsystems.MotorSubsystem;
import frc.trigon.robot.subsystems.climber.ClimberCommands;
import frc.trigon.robot.subsystems.climber.ClimberConstants;
import frc.trigon.robot.subsystems.elevator.ElevatorCommands;
import frc.trigon.robot.subsystems.elevator.ElevatorConstants;
import frc.trigon.robot.subsystems.intake.IntakeCommands;
import frc.trigon.robot.subsystems.intake.IntakeConstants;
import frc.trigon.robot.subsystems.ledstrip.LEDStripCommands;
import frc.trigon.robot.subsystems.ledstrip.LEDStripConstants;
import frc.trigon.robot.subsystems.pitcher.PitcherCommands;
import frc.trigon.robot.subsystems.shooter.ShooterCommands;
import frc.trigon.robot.subsystems.swerve.SwerveCommands;
import frc.trigon.robot.subsystems.transporter.TransporterCommands;
import frc.trigon.robot.subsystems.transporter.TransporterConstants;
import frc.trigon.robot.utilities.AllianceUtilities;
import frc.trigon.robot.utilities.ShootingCalculations;
import org.littletonrobotics.junction.Logger;

import java.awt.*;
import java.util.function.BooleanSupplier;
import java.util.function.Supplier;

public class Commands {
    public static boolean IS_BRAKING = true;
    private static final ShootingCalculations SHOOTING_CALCULATIONS = ShootingCalculations.getInstance();

    public static Command withoutRequirements(Command command) {
        return new FunctionalCommand(
                command::initialize,
                command::execute,
                command::end,
                command::isFinished
        );
    }

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
                new InstantCommand(() -> RobotContainer.ELEVATOR.setDidOpenElevator(true)),
                Commands.withoutRequirements(TransporterCommands.getSetTargetStateCommand(TransporterConstants.TransporterState.ALIGNING_FOR_AMP)).withTimeout(0.13).andThen(
                        ElevatorCommands.getSetTargetStateCommand(ElevatorConstants.ElevatorState.SCORE_AMP).alongWith(
                                runWhen(Commands.withoutRequirements(TransporterCommands.getSetTargetStateCommand(TransporterConstants.TransporterState.SCORE_AMP)), OperatorConstants.CONTINUE_TRIGGER)
                        )
                ),
                getContinuousConditionalCommand(LEDStripCommands.getStaticColorCommand(Color.green, LEDStripConstants.LED_STRIPS), LEDStripCommands.getStaticColorCommand(Color.red, LEDStripConstants.LED_STRIPS), Commands::atAmpPose),
                duplicate(CommandConstants.FACE_AMP_COMMAND)
        );
    }

    private static boolean atAmpPose() {
        final Pose2d currentPose = RobotContainer.POSE_ESTIMATOR.getCurrentPose().toMirroredAlliancePose();
        return
                Math.abs(currentPose.getRotation().getDegrees() - FieldConstants.IN_FRONT_OF_AMP_POSE.getRotation().getDegrees()) < 4 &&
                        Math.abs(currentPose.getX() - FieldConstants.IN_FRONT_OF_AMP_POSE.getX()) < 0.15 &&
                        Math.abs(currentPose.getY() - FieldConstants.IN_FRONT_OF_AMP_POSE.getY()) < 0.05;
    }

    public static Command getAutonomousScoreInAmpCommand() {
        return new SequentialCommandGroup(
                TransporterCommands.getSetTargetStateCommand(TransporterConstants.TransporterState.ALIGNING_FOR_AMP).withTimeout(0.13),
                ElevatorCommands.getSetTargetStateCommand(ElevatorConstants.ElevatorState.SCORE_AMP).alongWith(
                        runWhenContinueTriggerPressed(TransporterCommands.getSetTargetStateCommand(TransporterConstants.TransporterState.SCORE_AMP)),
                        getAutonomousDriveToAmpCommand().andThen(
                                duplicate(CommandConstants.FIELD_RELATIVE_DRIVE_COMMAND)
                        )
                )
        );
    }

    public static Command getShootAtSpeakerCommand() {
        return new ParallelCommandGroup(
                getPrepareShootingCommand(),
                runWhen(TransporterCommands.getSetTargetStateCommand(TransporterConstants.TransporterState.FEEDING), () -> RobotContainer.SHOOTER.atTargetShootingVelocity() && RobotContainer.PITCHER.atTargetPitch() && RobotContainer.SHOOTER.getTargetVelocityRevolutionsPerSecond() != 0 && RobotContainer.SWERVE.atAngle(AllianceUtilities.toMirroredAllianceRotation(SHOOTING_CALCULATIONS.calculateTargetRobotAngle())))
        );
    }

    public static Command getShootAtSpeakerTestingCommand() {
        return new ParallelCommandGroup(
                getPrepareShootingTestingCommand(),
                runWhen(TransporterCommands.getSetTargetStateCommand(TransporterConstants.TransporterState.FEEDING), () -> RobotContainer.SHOOTER.atTargetShootingVelocity() && RobotContainer.PITCHER.atTargetPitch())
        );
    }

    public static Command getResetPoseToAutoPoseCommand(Supplier<String> pathName) {
        return new InstantCommand(
                () -> {
                    if (DriverStation.isEnabled())
                        return;
                    final Pose2d autoStartPose = PathPlannerAuto.getStaringPoseFromAutoFile(pathName.get());
                    final AllianceUtilities.AlliancePose2d allianceAutoStartPose = AllianceUtilities.AlliancePose2d.fromBlueAlliancePose(AllianceUtilities.toMirroredAlliancePose(autoStartPose));
                    RobotContainer.POSE_ESTIMATOR.resetPose(allianceAutoStartPose);
                }
        ).ignoringDisable(true);
    }

    public static Command getPrepareShootingTestingCommand() {
        return new ParallelCommandGroup(
                PitcherCommands.getDebuggingCommand(),
                ShooterCommands.getSetTargetShootingVelocityCommand(-80)
//                LEDStripCommands.getStaticColorCommand(Color.PINK, LEDStripConstants.LED_STRIPS)
        );
    }

    public static Command getPrepareShootingCommand() {
        return new ParallelCommandGroup(
                getUpdateShootingCalculationsCommand(),
                PitcherCommands.getPitchToSpeakerCommand(),
                ShooterCommands.getShootAtSpeakerWithoutCurrentLimit(),
                SwerveCommands.getClosedLoopFieldRelativeDriveCommand(
                        () -> CommandConstants.calculateDriveStickAxisValue(OperatorConstants.DRIVER_CONTROLLER.getLeftY()),
                        () -> CommandConstants.calculateDriveStickAxisValue(OperatorConstants.DRIVER_CONTROLLER.getLeftX()),
                        SHOOTING_CALCULATIONS::calculateTargetRobotAngle
                )
//                LEDStripCommands.getStaticColorCommand(Color.PINK, LEDStripConstants.LED_STRIPS)
        );
    }

    public static Command getWarmShootingCommand() {
        return new ParallelCommandGroup(
                getUpdateShootingCalculationsCommand(),
                PitcherCommands.getPitchToSpeakerCommand(),
                ShooterCommands.getShootAtSpeakerWithCurrentLimit()
//                LEDStripCommands.getStaticColorCommand(Color.PINK, LEDStripConstants.LED_STRIPS)
        );
    }

    public static Command getEjectCommand() {
        return new ParallelCommandGroup(
                TransporterCommands.getSetTargetStateCommand(TransporterConstants.TransporterState.EJECTING),
                IntakeCommands.getSetTargetStateCommand(IntakeConstants.IntakeState.EJECTING)
        );
    }

    public static Command getDeliveryCommand() {
        return new ParallelCommandGroup(
                ShooterCommands.getSetTargetShootingVelocityCommand(-50),
                PitcherCommands.getSetTargetPitchCommand(Rotation2d.fromDegrees(53)),
                SwerveCommands.getClosedLoopFieldRelativeDriveCommand(
                        () -> CommandConstants.calculateDriveStickAxisValue(OperatorConstants.DRIVER_CONTROLLER.getLeftY()),
                        () -> CommandConstants.calculateDriveStickAxisValue(OperatorConstants.DRIVER_CONTROLLER.getLeftX()),
                        () -> Rotation2d.fromDegrees(-35)
                ),
                runWhen(TransporterCommands.getSetTargetStateCommand(TransporterConstants.TransporterState.FEEDING), () -> RobotContainer.SHOOTER.atTargetShootingVelocity() && RobotContainer.PITCHER.atTargetPitch())
        );
    }

    public static Command getPrepareShootingForAutoCommand() {
        return new ParallelCommandGroup(
                getUpdateShootingCalculationsCommand(),
                PitcherCommands.getPitchToSpeakerCommand(),
                ShooterCommands.getShootAtSpeakerWithoutCurrentLimit()
        );
    }

    public static Command getCloseShotCommand() {
        return new SequentialCommandGroup(
                getPrepareForCloseShotCommand().until(() -> RobotContainer.SHOOTER.atTargetShootingVelocity() && RobotContainer.PITCHER.atTargetPitch()),
                TransporterCommands.getSetTargetStateCommand(TransporterConstants.TransporterState.FEEDING).alongWith(getPrepareForCloseShotCommand())
        );
    }

    public static Command getPrepareForCloseShotCommand() {
        return new ParallelCommandGroup(
                ShooterCommands.getSetTargetShootingVelocityCommand(ShootingConstants.CLOSE_SHOT_VELOCITY_METERS_PER_SECOND),
                PitcherCommands.getSetTargetPitchCommand(ShootingConstants.CLOSE_SHOT_ANGLE)
        );
    }

    public static Command getClimbCommand() {
        return new SequentialCommandGroup(
                new InstantCommand(() -> {
                    CommandConstants.IS_CLIMBING = true;
                    Logger.recordOutput("IsClimbing", true);
                    RobotContainer.ELEVATOR.setDidOpenElevator(true);
                }),
                ClimberCommands.getSetTargetStateCommand(ClimberConstants.ClimberState.CLIMBING_PREPARATION).until(OperatorConstants.CONTINUE_TRIGGER).alongWith(TransporterCommands.getSetTargetStateCommand(TransporterConstants.TransporterState.ALIGNING_FOR_AMP).withTimeout(0.13)),
                ClimberCommands.getSetTargetStateCommand(ClimberConstants.ClimberState.CLIMB).alongWith(
                        runWhen(ElevatorCommands.getSetTargetStateCommand(ElevatorConstants.ElevatorState.SCORE_TRAP), RobotContainer.CLIMBER::isReadyForElevatorOpening),
                        new WaitUntilCommand(RobotContainer.ELEVATOR::isOpenForTrap).andThen(
                                TransporterCommands.getSetTargetStateCommand(TransporterConstants.TransporterState.ALIGNING_FOR_TRAP).withTimeout(0.25),
                                runWhen(TransporterCommands.getSetTargetStateCommand(TransporterConstants.TransporterState.SCORE_TRAP), OperatorConstants.SECOND_CONTINUE_TRIGGER)
                        )
                )
        ).until(OperatorConstants.CONTINUE_TRIGGER.and(() -> RobotContainer.ELEVATOR.getTargetState() == ElevatorConstants.ElevatorState.SCORE_TRAP)).andThen(
                new SequentialCommandGroup(
                        ClimberCommands.getSetTargetStateCommand(ClimberConstants.ClimberState.CLIMB_FINISH).alongWith(
                                TransporterCommands.getSetTargetStateCommand(TransporterConstants.TransporterState.STOPPED),
                                ElevatorCommands.getSetTargetStateCommand(ElevatorConstants.ElevatorState.SCORE_TRAP).until(() -> RobotContainer.CLIMBER.atTargetState() && RobotContainer.CLIMBER.getTargetState() == ClimberConstants.ClimberState.CLIMB_FINISH).andThen(
                                        ElevatorCommands.getSetTargetStateCommand(ElevatorConstants.ElevatorState.SCORE_TRAP_LOWERED)
                                )
                        ).until(() -> RobotContainer.ELEVATOR.atTargetState() && RobotContainer.ELEVATOR.getTargetState() == ElevatorConstants.ElevatorState.SCORE_TRAP_LOWERED),
                        ClimberCommands.getSetTargetStateCommand(ClimberConstants.ClimberState.CLIMB_MIDDLE).alongWith(
                                ElevatorCommands.getSetTargetStateCommand(ElevatorConstants.ElevatorState.SCORE_TRAP_LOWERED).until(() -> RobotContainer.CLIMBER.atTargetState() && RobotContainer.CLIMBER.getTargetState() == ClimberConstants.ClimberState.CLIMB_MIDDLE).andThen(
                                        ElevatorCommands.getSetTargetStateCommand(ElevatorConstants.ElevatorState.FINISH_TRAP)
                                )
                        )
                )
        );
    }

//    public static Command getClimbCommand() {
//        return new SequentialCommandGroup(
//                new InstantCommand(() -> {
//                    CommandConstants.IS_CLIMBING = true;
//                    Logger.recordOutput("IsClimbing", true);
//                }),
//                ClimberCommands.getSetTargetStateCommand(ClimberConstants.ClimberState.CLIMBING_PREPARATION).until(OperatorConstants.CONTINUE_TRIGGER).alongWith(TransporterCommands.getSetTargetStateCommand(TransporterConstants.TransporterState.ALIGNING_FOR_AMP).withTimeout(0.13)),
//                ClimberCommands.getSetTargetStateCommand(ClimberConstants.ClimberState.CLIMB)
//        );
//    }

    public static Command getNoteCollectionCommand() {
        return new ParallelCommandGroup(
                new AlignToNoteCommand().onlyIf(() -> CommandConstants.SHOULD_ALIGN_TO_NOTE),
                LEDStripCommands.getStaticColorCommand(Color.orange, LEDStripConstants.LED_STRIPS).asProxy().onlyIf(() -> !CommandConstants.SHOULD_ALIGN_TO_NOTE),
                getNonAssitedNoteCollectionCommand()
        );
    }

    public static Command getNonAssitedNoteCollectionCommand() {
        return new ParallelCommandGroup(
                IntakeCommands.getSetTargetStateCommand(IntakeConstants.IntakeState.COLLECTING),
                TransporterCommands.getSetTargetStateCommand(TransporterConstants.TransporterState.COLLECTING)
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

    private static Command getAutonomousDriveToAmpCommand() {
        return SwerveCommands.getDriveToPoseCommand(
                () -> AllianceUtilities.AlliancePose2d.fromBlueAlliancePose(FieldConstants.IN_FRONT_OF_AMP_POSE),
                AutonomousConstants.REAL_TIME_CONSTRAINTS
        );
    }

    private static Command runWhenContinueTriggerPressed(Command command) {
        return runWhen(command, OperatorConstants.CONTINUE_TRIGGER);
    }

    private static Command getUpdateShootingCalculationsCommand() {
        return new RunCommand(SHOOTING_CALCULATIONS::updateCalculations);
    }
}
