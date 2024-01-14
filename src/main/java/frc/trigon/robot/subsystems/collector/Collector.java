package frc.trigon.robot.subsystems.collector;

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

    @Override
    public void periodic() {
        collectorIO.updateInputs(collectorInputs);
        Logger.processInputs("Collector", collectorInputs);
    }

    @Override
    public void setBrake(boolean brake) {
        collectorIO.setBrake(brake);
    }

    @Override
    public void drive(Measure<Voltage> voltageMeasure) {
        collectorIO.setAngleMotorVoltage(voltageMeasure.in(Units.Volts));
    }

    @Override
    public void updateLog(SysIdRoutineLog log) {
        log.motor("Angle")
                .angularPosition(Units.Degrees.of(collectorInputs.angleMotorPositionDegrees))
                .angularVelocity(Units.DegreesPerSecond.of(collectorInputs.angleMotorVelocityDegreesPerSecond))
                .voltage(Units.Volts.of(collectorInputs.angleMotorVoltage));
    }

    @Override
    public SysIdRoutine.Config getSysIdConfig() {
        return CollectorConstants.SYSID_CONFIG;
    }

    @Override
    public void stop() {
        collectorIO.stopAngleMotor();
        collectorIO.stopCollectionMotor();
    }

    void setTargetState(CollectorConstants.CollectorState state) {
        collectorIO.setTargetAngle(state.angle);
        collectorIO.setCollectionVoltage(state.voltage);
    }
}