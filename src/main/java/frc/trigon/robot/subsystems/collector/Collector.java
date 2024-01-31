package frc.trigon.robot.subsystems.collector;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
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

public class Collector extends MotorSubsystem {
    private final CollectorInputsAutoLogged collectorInputs = new CollectorInputsAutoLogged();
    private final CollectorIO collectorIO = CollectorIO.generateIO();

    public Collector() {
        setName("Collector");
        Commands.getDelayedCommand(1, this::configureChangingDefaultCommand).schedule();
    }

    @Override
    public void periodic() {
        collectorIO.updateInputs(collectorInputs);
        Logger.processInputs("Collector", collectorInputs);
        updateMechanisms();
    }

    @Override
    public void setBrake(boolean brake) {
        collectorIO.setBrake(brake);
    }

    @Override
    public void drive(Measure<Voltage> voltageMeasure) {
        collectorIO.setTargetAngleMotorVoltage(voltageMeasure.in(Units.Volts));
    }

    @Override
    public void updateLog(SysIdRoutineLog log) {
        log.motor("CollectorAngle")
                .angularPosition(Units.Degrees.of(collectorInputs.anglePositionDegrees))
                .angularVelocity(Units.DegreesPerSecond.of(collectorInputs.angleVelocityDegreesPerSecond))
                .voltage(Units.Volts.of(collectorInputs.angleMotorVoltage));
    }

    @Override
    public SysIdRoutine.Config getSysIdConfig() {
        return CollectorConstants.SYSID_CONFIG;
    }

    @Override
    public void stop() {
        collectorIO.stop();
        CollectorConstants.SPEED_MECHANISM.setTargetVelocity(0);
    }

    public boolean isOpenForElevator() {
        return CollectorConstants.OPEN_FOR_ELEVATOR_DEGREES > collectorInputs.anglePositionDegrees;
    }

    void setTargetState(CollectorConstants.CollectorState targetState) {
        collectorIO.setTargetAngle(targetState.angle);
        collectorIO.setTargetCollectionVoltage(targetState.collectionVoltage);
        CollectorConstants.SPEED_MECHANISM.setTargetVelocity(targetState.collectionVoltage);
    }

    private void updateMechanisms() {
        CollectorConstants.SPEED_MECHANISM.updateMechanism(collectorInputs.collectionMotorVoltage);
        CollectorConstants.CURRENT_POSITION_COLLECTOR_LIGAMENT.setAngle(collectorInputs.anglePositionDegrees);
        CollectorConstants.TARGET_POSITION_COLLECTOR_LIGAMENT.setAngle(collectorInputs.angleMotorProfiledSetpointDegrees);
        Logger.recordOutput("Mechanisms/CollectorAngleMechanism", CollectorConstants.COLLECTOR_MECHANISM);
        Logger.recordOutput("Poses/Components/CollectorPose", getCollectorPose());
    }

    private Pose3d getCollectorPose() {
        final Transform3d collectorTransform = new Transform3d(
                new Translation3d(),
                new Rotation3d(0, edu.wpi.first.math.util.Units.degreesToRadians(-collectorInputs.anglePositionDegrees), 0)
        );
        return CollectorConstants.COLLECTOR_ORIGIN_POINT.transformBy(collectorTransform);
    }

    private void configureChangingDefaultCommand() {
        final Trigger shouldRestByDefaultTrigger = new Trigger(() -> RobotContainer.ELEVATOR.isClosed() && !CommandConstants.IS_CLIMBING);
        shouldRestByDefaultTrigger.onTrue(new InstantCommand(this::defaultToResting));
        shouldRestByDefaultTrigger.onFalse(new InstantCommand(this::defaultToOpening));
    }

    private void defaultToOpening() {
        changeDefaultCommand(CollectorCommands.getSetTargetStateCommand(CollectorConstants.CollectorState.OPENING));
    }

    private void defaultToResting() {
        changeDefaultCommand(CollectorCommands.getSetTargetStateCommand(CollectorConstants.CollectorState.RESTING));
    }
}