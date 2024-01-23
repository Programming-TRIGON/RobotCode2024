package frc.trigon.robot.subsystems.elevator;

import edu.wpi.first.units.Units;
import edu.wpi.first.wpilibj.sysid.SysIdRoutineLog;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.trigon.robot.subsystems.MotorSubsystem;
import org.littletonrobotics.junction.Logger;

public class Elevator extends MotorSubsystem {
    private final static Elevator INSTANCE = new Elevator();
    private final ElevatorIO elevatorIO = ElevatorIO.generateIO();
    private final ElevatorInputsAutoLogged elevatorInputs = new ElevatorInputsAutoLogged();
    private ElevatorConstants.ElevatorState targetState = ElevatorConstants.ElevatorState.STOPPED;

    public static Elevator getInstance() {
        return INSTANCE;
    }

    private Elevator() {
        setName("Elevator");
    }

    @Override
    public void periodic() {
        elevatorIO.updateInputs(elevatorInputs);
        Logger.processInputs("Elevator", elevatorInputs);
        updateMechanism();
    }

    @Override
    public void updateLog(SysIdRoutineLog log) {
        log.motor("Elevator")
                .linearPosition(Units.Meters.of(elevatorInputs.positionMeters))
                .linearVelocity(Units.MetersPerSecond.of(elevatorInputs.velocityMetersPerSecond))
                .voltage(Units.Volts.of(elevatorInputs.motorVoltage));
    }

    @Override
    public SysIdRoutine.Config getSysIdConfig() {
        return ElevatorConstants.SYSID_CONFIG;
    }

    @Override
    public void setBrake(boolean brake) {
        elevatorIO.setBrake(brake);
    }

    @Override
    public void stop() {
        elevatorIO.stop();
    }

    public boolean atTargetState() {
        return this.targetState.positionMeters == elevatorInputs.positionMeters;
    }

    void setTargetState(ElevatorConstants.ElevatorState targetState) {
        this.targetState = targetState;

        elevatorIO.setTargetPosition(targetState.positionMeters);
    }

    private void updateMechanism() {
        ElevatorConstants.ELEVATOR_LIGAMENT.setLength(elevatorInputs.positionMeters + ElevatorConstants.RETRACTED_ELEVATOR_LENGTH_METERS);
        ElevatorConstants.TARGET_ELEVATOR_POSITION_LIGAMENT.setLength(elevatorInputs.profiledSetpointMeters + ElevatorConstants.RETRACTED_ELEVATOR_LENGTH_METERS);
        Logger.recordOutput("Elevator/ElevatorMechanism", ElevatorConstants.ELEVATOR_MECHANISM);
    }
}