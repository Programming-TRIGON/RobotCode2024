package frc.trigon.robot.subsystems.elevator;

import frc.trigon.robot.subsystems.MotorSubsystem;
import org.littletonrobotics.junction.Logger;

public class Elevator extends MotorSubsystem {
    private final static Elevator INSTANCE = new Elevator();
    private final ElevatorIO elevatorIO = ElevatorIO.generateIO();
    private final ElevatorInputsAutoLogged elevatorInputs = new ElevatorInputsAutoLogged();
    private ElevatorConstants.ElevatorState targetState = ElevatorConstants.ElevatorState.BOTTOM;

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
    public void setBrake(boolean brake) {
        elevatorIO.setBrake(brake);
    }
    
    @Override
    public void stop() {
        elevatorIO.stopMotors();
    }

    void setTargetState(ElevatorConstants.ElevatorState targetState) {
        this.targetState = targetState;

        ElevatorConstants.TARGET_ELEVATOR_POSITION_LIGAMENT.setLength(elevatorInputs.profiledSetpoint + ElevatorConstants.RETRACTED_ELEVATOR_LENGTH_METERS);
        elevatorIO.setTargetPosition(elevatorInputs.profiledSetpoint);
    }

    private void updateMechanism() {
        ElevatorConstants.ELEVATOR_LIGAMENT.setLength(elevatorInputs.motorPositionMeters + ElevatorConstants.RETRACTED_ELEVATOR_LENGTH_METERS);
        Logger.recordOutput("Elevator/ElevatorMechanism", ElevatorConstants.ELEVATOR_MECHANISM);
    }
}