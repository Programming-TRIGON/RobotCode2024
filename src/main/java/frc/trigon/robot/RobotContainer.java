// Copyright (c) FIRST and other WPILib contributors.

// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.trigon.robot;

import com.pathplanner.lib.auto.AutoBuilder;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.trigon.robot.commands.Commands;
import frc.trigon.robot.components.KeyboardController;
import frc.trigon.robot.constants.CommandConstants;
import frc.trigon.robot.constants.OperatorConstants;
import frc.trigon.robot.poseestimation.poseestimator.PoseEstimator;
import frc.trigon.robot.subsystems.elevator.Elevator;
import frc.trigon.robot.subsystems.elevator.ElevatorCommands;
import frc.trigon.robot.subsystems.elevator.ElevatorConstants;
import frc.trigon.robot.subsystems.swerve.Swerve;
import org.littletonrobotics.junction.networktables.LoggedDashboardChooser;

public class RobotContainer {
    public static final PoseEstimator POSE_ESTIMATOR = new PoseEstimator();
    private final Swerve swerve = Swerve.getInstance();
    private final Elevator elevator = Elevator.getInstance();
    private final KeyboardController keyboard = new KeyboardController();
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
    }

    private void bindDefaultCommands() {
        swerve.setDefaultCommand(CommandConstants.FIELD_RELATIVE_DRIVE_COMMAND);
    }

    private void bindControllerCommands() {
        OperatorConstants.RESET_HEADING_TRIGGER.onTrue(CommandConstants.RESET_HEADING_COMMAND);
        OperatorConstants.DRIVE_FROM_DPAD_TRIGGER.whileTrue(CommandConstants.SELF_RELATIVE_DRIVE_FROM_DPAD_COMMAND);
        OperatorConstants.TOGGLE_FIELD_AND_SELF_RELATIVE_DRIVE_TRIGGER.onTrue(Commands.getToggleFieldAndSelfRelativeDriveCommand());
        OperatorConstants.TOGGLE_BRAKE_TRIGGER.onTrue(Commands.getToggleBrakeCommand());
        elevator.setDefaultCommand(ElevatorCommands.getSetTargetStateCommand(ElevatorConstants.ElevatorState.STOPPED));
        keyboard.a().whileTrue(ElevatorCommands.getSetTargetStateCommand(ElevatorConstants.ElevatorState.FEEDING));
        keyboard.b().whileTrue(ElevatorCommands.getSetTargetStateCommand(ElevatorConstants.ElevatorState.SCORE_AMP));
        keyboard.c().whileTrue(ElevatorCommands.getSetTargetStateCommand(ElevatorConstants.ElevatorState.SCORE_TRAP));
        keyboard.d().whileTrue(ElevatorCommands.getSetTargetStateCommand(ElevatorConstants.ElevatorState.COLLECTION));
//        keyboard.z().whileTrue(elevator.getQuasistaticCharacterizationCommand(SysIdRoutine.Direction.kForward));
//        keyboard.x().whileTrue(elevator.getQuasistaticCharacterizationCommand(SysIdRoutine.Direction.kReverse));
//        keyboard.a().whileTrue(elevator.getDynamicCharacterizationCommand(SysIdRoutine.Direction.kForward));
//        keyboard.s().whileTrue(elevator.getDynamicCharacterizationCommand(SysIdRoutine.Direction.kReverse));
    }

    private void buildAutoChooser() {
        autoChooser = new LoggedDashboardChooser<>("AutoChooser", AutoBuilder.buildAutoChooser());
    }
}
