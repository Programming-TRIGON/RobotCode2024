// Copyright (c) FIRST and other WPILib contributors.

// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.trigon.robot;

import com.pathplanner.lib.pathfinding.Pathfinding;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.trigon.robot.constants.FieldConstants;
import frc.trigon.robot.constants.RobotConstants;
import frc.trigon.robot.simulation.MotorSimulation;
import frc.trigon.robot.utilities.CurrentWatcher;
import frc.trigon.robot.utilities.LocalADStarAK;
import org.littletonrobotics.junction.LogFileUtil;
import org.littletonrobotics.junction.LoggedRobot;
import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.networktables.NT4Publisher;
import org.littletonrobotics.junction.wpilog.WPILOGReader;
import org.littletonrobotics.junction.wpilog.WPILOGWriter;

public class Robot extends LoggedRobot {
    public static final boolean IS_REAL = Robot.isReal();
    private final CommandScheduler commandScheduler = CommandScheduler.getInstance();
    private Command autonomousCommand;
    private RobotContainer robotContainer;

    @Override
    public void robotInit() {
        Pathfinding.setPathfinder(new LocalADStarAK());
        configLogger();
        robotContainer = new RobotContainer();
    }

    double speakerZ = 2.5;

    private void update() {
        final Pose2d mirroredAlliancePose = RobotContainer.POSE_ESTIMATOR.getCurrentPose().toMirroredAlliancePose();
        final double distanceToSpeaker = Math.abs(mirroredAlliancePose.getTranslation().getX() - FieldConstants.SPEAKER_TRANSLATION.getX());
        Logger.recordOutput("Distance", distanceToSpeaker);
        Logger.recordOutput("AdvisedAngle", Math.toDegrees(Math.atan2(speakerZ, distanceToSpeaker)));
    }

    @Override
    public void robotPeriodic() {
        commandScheduler.run();
        updatePeriodics();
        update();
    }

    @Override
    public void autonomousInit() {
        autonomousCommand = robotContainer.getAutonomousCommand();

        if (autonomousCommand != null)
            autonomousCommand.schedule();
    }

    @Override
    public void teleopInit() {
        if (autonomousCommand != null)
            autonomousCommand.cancel();
    }

    @Override
    public void testInit() {
        commandScheduler.cancelAll();
    }

    @Override
    public void simulationPeriodic() {
        MotorSimulation.updateRegisteredSimulations();
    }

    @Override
    public void disabledPeriodic() {
    }

    @Override
    public void autonomousPeriodic() {
    }

    @Override
    public void teleopPeriodic() {
    }

    @Override
    public void testPeriodic() {
    }

    private void updatePeriodics() {
        CurrentWatcher.checkCurrentForRegisteredWatchers();
        RobotContainer.POSE_ESTIMATOR.periodic();
    }

    private void configLogger() {
        if (RobotConstants.IS_REPLAY) {
            setUseTiming(false);
            final String logPath = LogFileUtil.findReplayLog();
            final String logWriterPath = LogFileUtil.addPathSuffix(logPath, "_replay");

            Logger.setReplaySource(new WPILOGReader(logPath));
            Logger.addDataReceiver(new WPILOGWriter(logWriterPath));
        } else {
            Logger.addDataReceiver(new NT4Publisher());
            Logger.addDataReceiver(new WPILOGWriter(RobotConstants.ROBOT_TYPE.loggingPath));
        }

        Logger.start();
    }
}
