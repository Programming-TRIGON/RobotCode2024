// Copyright (c) FIRST and other WPILib contributors.

// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.trigon.robot;

import com.pathplanner.lib.auto.AutoBuilder;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.trigon.robot.commands.Commands;
import frc.trigon.robot.commands.LEDAutoSetupCommand;
import frc.trigon.robot.constants.AutonomousConstants;
import frc.trigon.robot.constants.CameraConstants;
import frc.trigon.robot.constants.CommandConstants;
import frc.trigon.robot.constants.OperatorConstants;
import frc.trigon.robot.poseestimation.poseestimator.PoseEstimator;
import frc.trigon.robot.subsystems.MotorSubsystem;
import frc.trigon.robot.subsystems.climber.Climber;
import frc.trigon.robot.subsystems.climber.ClimberCommands;
import frc.trigon.robot.subsystems.elevator.Elevator;
import frc.trigon.robot.subsystems.elevator.ElevatorCommands;
import frc.trigon.robot.subsystems.elevator.ElevatorConstants;
import frc.trigon.robot.subsystems.intake.Intake;
import frc.trigon.robot.subsystems.intake.IntakeCommands;
import frc.trigon.robot.subsystems.intake.IntakeConstants;
import frc.trigon.robot.subsystems.ledstrip.LEDStrip;
import frc.trigon.robot.subsystems.ledstrip.LEDStripCommands;
import frc.trigon.robot.subsystems.pitcher.Pitcher;
import frc.trigon.robot.subsystems.roller.Roller;
import frc.trigon.robot.subsystems.roller.RollerCommands;
import frc.trigon.robot.subsystems.roller.RollerConstants;
import frc.trigon.robot.subsystems.shooter.Shooter;
import frc.trigon.robot.subsystems.shooter.ShooterCommands;
import frc.trigon.robot.subsystems.swerve.Swerve;
import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.networktables.LoggedDashboardChooser;

import java.awt.*;

public class RobotContainer {
    public static final Swerve SWERVE = new Swerve();
    public static final Shooter SHOOTER = new Shooter();
    public static final Pitcher PITCHER = new Pitcher();
    public static final Intake INTAKE = new Intake();
    public static final Elevator ELEVATOR = new Elevator();
    public static final Roller ROLLER = new Roller();
    public static final Climber CLIMBER = new Climber();
    public static final PoseEstimator POSE_ESTIMATOR = new PoseEstimator(
            CameraConstants.REAR_LEFT_CAMERA,
            CameraConstants.REAR_RIGHT_CAMERA,
            CameraConstants.FRONT_MIDDLE_CAMERA,
            CameraConstants.REAR_MIDDLE_CAMERA
    );
    private final LoggedDashboardChooser<Command> autoChooser;

    public RobotContainer() {
        AutonomousConstants.init();
        autoChooser = new LoggedDashboardChooser<>("AutoChooser", AutoBuilder.buildAutoChooser("Picture3"));
        configureBindings();
        DriverStation.silenceJoystickConnectionWarning(true);
        Logger.recordOutput("ShouldAlignToNote", false);
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
        SWERVE.setDefaultCommand(CommandConstants.FIELD_RELATIVE_DRIVE_COMMAND);
        SHOOTER.setDefaultCommand(ShooterCommands.getStopShootingCommand());
        INTAKE.setDefaultCommand(IntakeCommands.getSetTargetStateCommand(IntakeConstants.IntakeState.STOPPED));
        PITCHER.setDefaultCommand(CommandConstants.PITCHER_RESTING_COMMAND);
        ELEVATOR.setDefaultCommand(ElevatorCommands.getSetTargetStateCommand(ElevatorConstants.ElevatorState.RESTING));
        ROLLER.setDefaultCommand(RollerCommands.getSetTargetStateCommand(RollerConstants.RollerState.STOPPED));
        CLIMBER.setDefaultCommand(ClimberCommands.getStopCommand());
        LEDStrip.setDefaultCommandForAllLEDS((ledStrip) -> LEDStripCommands.getAnimateColorFlowCommand(new Color(0, 150, 255), 0.5, ledStrip));
//        LEDStrip.setDefaultCommandForAllLEDS((ledStrip) -> LEDStripCommands.getAnimateFireCommand(1, 0.01, 0.8, 0.1, ledStrip));
    }

    private void bindControllerCommands() {
        OperatorConstants.RESET_HEADING_TRIGGER.onTrue(CommandConstants.RESET_HEADING_COMMAND);
        OperatorConstants.DRIVE_FROM_DPAD_TRIGGER.whileTrue(CommandConstants.SELF_RELATIVE_DRIVE_FROM_DPAD_COMMAND);
        OperatorConstants.TOGGLE_FIELD_AND_SELF_RELATIVE_DRIVE_TRIGGER.onTrue(Commands.getToggleFieldAndSelfRelativeDriveCommand());
        OperatorConstants.TOGGLE_BRAKE_TRIGGER.onTrue(Commands.getToggleBrakeCommand());

        OperatorConstants.SECOND_CONTINUE_TRIGGER.or(OperatorConstants.SHOOT_AT_SPEAKER_TRIGGER).whileTrue(Commands.getShootAtSpeakerCommand());
        OperatorConstants.CLIMB_TRIGGER.whileTrue(Commands.getClimbCommand());
        OperatorConstants.SCORE_IN_AMP_TRIGGER.whileTrue(Commands.getScoreInAmpCommand());
        OperatorConstants.AUTONOMOUS_SCORE_IN_AMP_TRIGGER.whileTrue(Commands.getAutonomousScoreInAmpCommand());
        OperatorConstants.COLLECT_TRIGGER.whileTrue(Commands.getNoteCollectionCommand());
        OperatorConstants.FACE_AMP_TRIGGER.whileTrue(CommandConstants.FACE_AMP_COMMAND);
        OperatorConstants.FACE_SPEAKER_TRIGGER.whileTrue(CommandConstants.FACE_SPEAKER_COMMAND);
        OperatorConstants.CLOSE_SHOT_TRIGGER.whileTrue(Commands.getCloseShotCommand());
        OperatorConstants.LED_AUTO_SETUP_TRIGGER.toggleOnTrue(new LEDAutoSetupCommand(() -> autoChooser.get().getName()));
        OperatorConstants.EJECT_NOTE_TRIGGER.whileTrue(RollerCommands.getSetTargetStateCommand(RollerConstants.RollerState.EJECTING));
        OperatorConstants.MOVE_CLIMBER_DOWN_MANUALLY_TRIGGER.whileTrue(CommandConstants.MOVE_CLIMBER_DOWN_MANUALLY_COMMAND);
        OperatorConstants.MOVE_CLIMBER_UP_MANUALLY_TRIGGER.whileTrue(CommandConstants.MOVE_CLIMBER_UP_MANUALLY_COMMAND);
//        OperatorConstants.DEBUG_BUTTON.whileTrue(PitcherCommands.getDebuggingCommand());

        OperatorConstants.AMPLIFY_LEDS_TRIGGER.toggleOnTrue(CommandConstants.AMPLIFY_LEDS_COMMAND);
        OperatorConstants.RESET_AUTO_POSE_TRIGGER.onTrue(Commands.getResetPoseToAutoPoseCommand(() -> autoChooser.get().getName()));
        OperatorConstants.OVERRIDE_IS_CLIMBING_TRIGGER.onTrue(CommandConstants.OVERRIDE_IS_CLIMBING_COMMAND);
        OperatorConstants.TURN_AUTOMATIC_NOTE_ALIGNING_ON_TRIGGER.onTrue(CommandConstants.TURN_AUTOMATIC_NOTE_ALIGNING_ON_COMMAND);
        OperatorConstants.TURN_AUTOMATIC_NOTE_ALIGNING_OFF_TRIGGER.onTrue(CommandConstants.TURN_AUTOMATIC_NOTE_ALIGNING_OFF_COMMAND);
    }

    private void configureSysIdBindings(MotorSubsystem subsystem) {
        OperatorConstants.FORWARD_QUASISTATIC_CHARACTERIZATION_TRIGGER.whileTrue(subsystem.getQuasistaticCharacterizationCommand(SysIdRoutine.Direction.kForward));
        OperatorConstants.BACKWARD_QUASISTATIC_CHARACTERIZATION_TRIGGER.whileTrue(subsystem.getQuasistaticCharacterizationCommand(SysIdRoutine.Direction.kReverse));
        OperatorConstants.FORWARD_DYNAMIC_CHARACTERIZATION_TRIGGER.whileTrue(subsystem.getDynamicCharacterizationCommand(SysIdRoutine.Direction.kForward));
        OperatorConstants.BACKWARD_DYNAMIC_CHARACTERIZATION_TRIGGER.whileTrue(subsystem.getDynamicCharacterizationCommand(SysIdRoutine.Direction.kReverse));
        subsystem.setDefaultCommand(edu.wpi.first.wpilibj2.command.Commands.idle(subsystem));
    }
}