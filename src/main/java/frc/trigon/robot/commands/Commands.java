package frc.trigon.robot.commands;

import edu.wpi.first.wpilibj2.command.*;
import frc.trigon.robot.constants.CommandConstants;
import frc.trigon.robot.constants.OperatorConstants;
import frc.trigon.robot.subsystems.MotorSubsystem;
import frc.trigon.robot.subsystems.climber.ClimberCommands;
import frc.trigon.robot.subsystems.climber.ClimberConstants;
import frc.trigon.robot.subsystems.collector.Collector;
import frc.trigon.robot.subsystems.collector.CollectorCommands;
import frc.trigon.robot.subsystems.collector.CollectorConstants;
import frc.trigon.robot.subsystems.elevator.ElevatorCommands;
import frc.trigon.robot.subsystems.elevator.ElevatorConstants;
import frc.trigon.robot.subsystems.pitcher.PitcherCommands;
import frc.trigon.robot.subsystems.roller.RollerCommands;
import frc.trigon.robot.subsystems.roller.RollerConstants;
import frc.trigon.robot.subsystems.shooter.ShooterCommands;
import frc.trigon.robot.subsystems.swerve.Swerve;
import frc.trigon.robot.subsystems.swerve.SwerveCommands;
import frc.trigon.robot.utilities.ShootingCalculations;

import java.util.function.BooleanSupplier;

public class Commands {
    private static final ShootingCalculations SHOOTING_CALCULATIONS = ShootingCalculations.getInstance();
    private static final Collector COLLECTOR = Collector.getInstance();
    private static boolean IS_BRAKING = true;

    /**
     * @return a command that toggles between the swerve's default command, from field relative to self relative
     */
    public static Command getToggleFieldAndSelfRelativeDriveCommand() {
        return new InstantCommand(() -> {
            if (Swerve.getInstance().getDefaultCommand().equals(CommandConstants.FIELD_RELATIVE_DRIVE_COMMAND))
                Swerve.getInstance().setDefaultCommand(CommandConstants.SELF_RELATIVE_DRIVE_COMMAND);
            else
                Swerve.getInstance().setDefaultCommand(CommandConstants.FIELD_RELATIVE_DRIVE_COMMAND);

            Swerve.getInstance().getDefaultCommand().schedule();
        });
    }

    public static Command getToggleBrakeCommand() {
        return new InstantCommand(() -> {
            IS_BRAKING = !IS_BRAKING;
            MotorSubsystem.setAllSubsystemsBrakeAsync(IS_BRAKING);
        }).ignoringDisable(true);
    }

    public static Command getDelayedCommand(double delaySeconds, Runnable toRun) {
        return new WaitCommand(delaySeconds).andThen(toRun).ignoringDisable(true);
    }

    public static Command getScoreInAmpCommand() {
        return new ParallelCommandGroup(
                CollectorCommands.getSetTargetStateCommand(CollectorConstants.CollectorState.OPENING),
                runWhen(ElevatorCommands.getSetTargetStateCommand(ElevatorConstants.ElevatorState.SCORE_AMP), COLLECTOR::isOpenForElevator),
                runWhenContinueTriggerPressed(RollerCommands.getSetTargetStateCommand(RollerConstants.RollerState.SCORE_AMP))
        );
    }

    public static Command getShootAtSpeakerCommand() {
        return new ParallelCommandGroup(
                getPrepareShootingCommand(),
                runWhenContinueTriggerPressed(RollerCommands.getSetTargetStateCommand(RollerConstants.RollerState.FEEDING))
        );
    }

    public static Command getClimbCommand() {
        return new ParallelCommandGroup(
                CollectorCommands.getSetTargetStateCommand(CollectorConstants.CollectorState.OPENING),
                ClimberCommands.getSetTargetStateCommand(ClimberConstants.ClimberState.RAISED).until(OperatorConstants.CONTINUE_TRIGGER).andThen(
                        ClimberCommands.getSetTargetStateCommand(ClimberConstants.ClimberState.LOWERED)
                )
        );
    }

    public static Command getCollectNoteCommand() {
        return new ParallelCommandGroup(
                runWhen(ElevatorCommands.getSetTargetStateCommand(ElevatorConstants.ElevatorState.RESTING), COLLECTOR::isOpenForElevator),
                CollectorCommands.getSetTargetStateCommand(CollectorConstants.CollectorState.COLLECTING),
                RollerCommands.getSetTargetStateCommand(RollerConstants.RollerState.COLLECTING)
        );
    }

    public static Command getPrepareShootingCommand() {
        return new ParallelCommandGroup(
                getUpdateShootingCalculationsCommand(),
                PitcherCommands.getPitchToSpeakerCommand(),
                ShooterCommands.getShootAtSpeakerCommand(),
                SwerveCommands.getClosedLoopFieldRelativeDriveCommand(
                        () -> CommandConstants.calculateDriveStickAxisValue(OperatorConstants.DRIVER_CONTROLLER.getLeftY()),
                        () -> CommandConstants.calculateDriveStickAxisValue(OperatorConstants.DRIVER_CONTROLLER.getLeftX()),
                        SHOOTING_CALCULATIONS::calculateTargetRobotAngle
                )
        );
    }

    private static Command runWhenContinueTriggerPressed(Command command) {
        return runWhen(command, OperatorConstants.CONTINUE_TRIGGER);
    }

    private static Command runWhen(Command command, BooleanSupplier condition) {
        return command.onlyIf(condition).repeatedly();
    }

    private static Command getUpdateShootingCalculationsCommand() {
        return new RunCommand(SHOOTING_CALCULATIONS::updateCalculations);
    }
}
