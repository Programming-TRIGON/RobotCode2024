// Copyright (c) FIRST and other WPILib contributors.

// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.trigon.robot;

import com.pathplanner.lib.auto.AutoBuilder;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.wpilibj2.command.Command;
import frc.trigon.robot.commands.Commands;
import frc.trigon.robot.constants.CommandConstants;
import frc.trigon.robot.constants.OperatorConstants;
import frc.trigon.robot.poseestimation.poseestimator.PoseEstimator;
import frc.trigon.robot.subsystems.climber.Climber;
import frc.trigon.robot.subsystems.climber.ClimberCommands;
import frc.trigon.robot.subsystems.climber.ClimberConstants;
import frc.trigon.robot.subsystems.collector.Collector;
import frc.trigon.robot.subsystems.collector.CollectorCommands;
import frc.trigon.robot.subsystems.collector.CollectorConstants;
import frc.trigon.robot.subsystems.elevator.Elevator;
import frc.trigon.robot.subsystems.elevator.ElevatorCommands;
import frc.trigon.robot.subsystems.elevator.ElevatorConstants;
import frc.trigon.robot.subsystems.pitcher.Pitcher;
import frc.trigon.robot.subsystems.roller.Roller;
import frc.trigon.robot.subsystems.roller.RollerCommands;
import frc.trigon.robot.subsystems.roller.RollerConstants;
import frc.trigon.robot.subsystems.shooter.Shooter;
import frc.trigon.robot.subsystems.shooter.ShooterCommands;
import frc.trigon.robot.subsystems.swerve.Swerve;
import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.networktables.LoggedDashboardChooser;

public class RobotContainer {
    public static final PoseEstimator POSE_ESTIMATOR = new PoseEstimator();
    private final Swerve swerve = Swerve.getInstance();
    private final Shooter shooter = Shooter.getInstance();
    private final Pitcher pitcher = Pitcher.getInstance();
    private final Collector collector = Collector.getInstance();
    private final Roller roller = Roller.getInstance();
    private final Elevator elevator = Elevator.getInstance();
    private final Climber climber = Climber.getInstance();
    private LoggedDashboardChooser<Command> autoChooser;

    public RobotContainer() {
        configureBindings();
        buildAutoChooser();
    }

    /**
     * @return the command to run in autonomous mode
     */
    public Command getAutonomousCommand() {
        return autoChooser.get();
    }

    private void configureBindings() {
        bindDefaultCommands();
        bindControllerCommands();
        Logger.recordOutput("Empty", new Pose3d());
    }

    private void bindDefaultCommands() {
        swerve.setDefaultCommand(CommandConstants.FIELD_RELATIVE_DRIVE_COMMAND);
        shooter.setDefaultCommand(ShooterCommands.getStopShootingCommand());
        collector.setDefaultCommand(CollectorCommands.getSetTargetStateCommand(CollectorConstants.CollectorState.RESTING));
        pitcher.setDefaultCommand(CommandConstants.PITCHER_RESTING_COMMAND);
        elevator.setDefaultCommand(ElevatorCommands.getSetTargetStateCommand(ElevatorConstants.ElevatorState.RESTING));
        roller.setDefaultCommand(RollerCommands.getSetTargetStateCommand(RollerConstants.RollerState.STOPPED));
        climber.setDefaultCommand(ClimberCommands.getSetTargetStateCommand(ClimberConstants.ClimberState.LOWERED));
    }

    private void bindControllerCommands() {
        OperatorConstants.RESET_HEADING_TRIGGER.onTrue(CommandConstants.RESET_HEADING_COMMAND);
        OperatorConstants.DRIVE_FROM_DPAD_TRIGGER.whileTrue(CommandConstants.SELF_RELATIVE_DRIVE_FROM_DPAD_COMMAND);
        OperatorConstants.TOGGLE_FIELD_AND_SELF_RELATIVE_DRIVE_TRIGGER.onTrue(Commands.getToggleFieldAndSelfRelativeDriveCommand());
        OperatorConstants.TOGGLE_BRAKE_TRIGGER.onTrue(Commands.getToggleBrakeCommand());

        OperatorConstants.SHOOT_AT_SPEAKER_TRIGGER.whileTrue(Commands.getShootAtSpeakerCommand());
        OperatorConstants.CLIMB_TRIGGER.whileTrue(Commands.getClimbCommand());
        OperatorConstants.SCORE_IN_AMP_TRIGGER.whileTrue(Commands.getScoreInAmpCommand());
        OperatorConstants.COLLECT_TRIGGER.whileTrue(Commands.getCollectNoteCommand());
    }

    private void buildAutoChooser() {
        autoChooser = new LoggedDashboardChooser<>("AutoChooser", AutoBuilder.buildAutoChooser());
    }
}