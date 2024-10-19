// Copyright (c) FIRST and other WPILib contributors.

// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.trigon.robot;

import com.pathplanner.lib.pathfinding.Pathfinding;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.trigon.robot.constants.CameraConstants;
import frc.trigon.robot.constants.OperatorConstants;
import frc.trigon.robot.constants.RobotConstants;
import org.littletonrobotics.junction.LogFileUtil;
import org.littletonrobotics.junction.LoggedRobot;
import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.networktables.NT4Publisher;
import org.littletonrobotics.junction.wpilog.WPILOGReader;
import org.littletonrobotics.junction.wpilog.WPILOGWriter;
import org.trigon.hardware.RobotHardwareStats;
import org.trigon.utilities.LocalADStarAK;

public class Robot extends LoggedRobot {
    public static final boolean IS_REAL = Robot.isReal();
    private final CommandScheduler commandScheduler = CommandScheduler.getInstance();
    private Command autonomousCommand;
    private RobotContainer robotContainer;
    private Pose2d pose1 = null, pose2 = null;

    @Override
    public void robotInit() {
        RobotConstants.init();
        Pathfinding.setPathfinder(new LocalADStarAK());
        configLogger();
        robotContainer = new RobotContainer();
        OperatorConstants.OPERATOR_CONTROLLER.numpad0().onTrue(new InstantCommand(
                () -> {
                    CameraConstants.REAR_MIDDLE_CAMERA.update();
                    pose1 = CameraConstants.REAR_MIDDLE_CAMERA.getEstimatedRobotPose().transformBy(new Transform2d(new Translation2d(), Rotation2d.fromDegrees(180)));
                    System.out.println("Pose 1: " + pose1);
                }
        ));
        OperatorConstants.OPERATOR_CONTROLLER.numpad1().onTrue(new InstantCommand(
                () -> {
                    CameraConstants.REAR_MIDDLE_CAMERA.update();
                    pose2 = CameraConstants.REAR_MIDDLE_CAMERA.getEstimatedRobotPose().transformBy(new Transform2d(new Translation2d(), Rotation2d.fromDegrees(180)));
                    System.out.println("Pose 2: " + pose2);
                }
        ));
        OperatorConstants.OPERATOR_CONTROLLER.numpad2().onTrue(new InstantCommand(
                () -> {
                    final Translation2d translation2d = calc(
                            pose1.getRotation().getRadians(), pose2.getRotation().getRadians(),
                            pose1.getX(), pose2.getX(),
                            pose1.getY(), pose2.getY()
                    );
                    System.out.println("__________________________" + translation2d);
                    Logger.recordOutput("TranslationX", translation2d.getX());
                    Logger.recordOutput("TranslationY", translation2d.getY());
                }
        ));
    }

    @Override
    public void robotPeriodic() {
        commandScheduler.run();
    }

    public static Translation2d calc(double a1, double a2, double x1, double x2, double y1, double y2) {
        return new Translation2d(
                calculateX(a1, a2, x1, x2, y1, y2),
                calculateY(a1, a2, x1, x2, y1, y2)
        );
    }

    public static double calculateX(double a1, double a2, double x1, double x2, double y1, double y2) {
        double A = Math.cos(a1) - Math.cos(a2);
        double B = Math.sin(a1) - Math.sin(a2);
        double C = x2 - x1;
        double D = y2 - y1;

        return (C * A + D * B) / (A * A + B * B);
    }

    public static double calculateY(double a1, double a2, double x1, double x2, double y1, double y2) {
        double A = Math.cos(a1) - Math.cos(a2);
        double B = Math.sin(a1) - Math.sin(a2);
        double C = x2 - x1;
        double D = y2 - y1;

        return (D * A - C * B) / (A * A + B * B);
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
    }

    @Override
    public void disabledPeriodic() {
    }

    @Override
    public void autonomousPeriodic() {
//        REVPhysicsSim.getInstance().run();
    }

    @Override
    public void teleopPeriodic() {
    }

    @Override
    public void testPeriodic() {
    }

    private void configLogger() {
        if (RobotHardwareStats.isReplay()) {
            setUseTiming(false);
            final String logPath = LogFileUtil.findReplayLog();
            final String logWriterPath = LogFileUtil.addPathSuffix(logPath, "_replay");

            Logger.setReplaySource(new WPILOGReader(logPath));
            Logger.addDataReceiver(new WPILOGWriter(logWriterPath));
        } else {
            Logger.addDataReceiver(new NT4Publisher());
            Logger.addDataReceiver(new WPILOGWriter(RobotConstants.LOGGING_PATH));
        }

        Logger.start();
    }
}
