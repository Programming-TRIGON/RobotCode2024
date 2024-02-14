package frc.trigon.robot.subsystems.intake;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.StartEndCommand;
import frc.trigon.robot.RobotContainer;

public class IntakeCommands {
    private static final Intake INTAKE = RobotContainer.INTAKE;

    public static Command getSetTargetStateCommand(IntakeConstants.IntakeState targetState) {
        return getSetTargetStateCommand(targetState.angle, targetState.collectionVoltage);
    }

    public static Command getSetTargetStateCommand(Rotation2d targetAngle, double collectorVoltage) {
        return new StartEndCommand(
                () -> INTAKE.setTargetState(targetAngle, collectorVoltage),
                INTAKE::stop,
                INTAKE
        );
    }

    public static Command getSetTargetAngleCommand(Rotation2d targetAngle) {
        return new StartEndCommand(
                () -> INTAKE.setTargetState(targetAngle, 0),
                INTAKE::stop,
                INTAKE
        );
    }
}
