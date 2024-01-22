package frc.trigon.robot.subsystems.collector;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.units.Measure;
import edu.wpi.first.units.Units;
import edu.wpi.first.units.Voltage;
import edu.wpi.first.wpilibj.sysid.SysIdRoutineLog;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.trigon.robot.subsystems.MotorSubsystem;
import org.littletonrobotics.junction.Logger;

public class Collector extends MotorSubsystem {
    private final static Collector INSTANCE = new Collector();
    private final CollectorInputsAutoLogged collectorInputs = new CollectorInputsAutoLogged();
    private final CollectorIO collectorIO = CollectorIO.generateIO();

    public static Collector getInstance() {
        return INSTANCE;
    }

    private Collector() {
        setName("Collector");
        Logger.recordOutput("CollectorPose", new Pose3d());
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
        log.motor("Angle")
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
    }
}