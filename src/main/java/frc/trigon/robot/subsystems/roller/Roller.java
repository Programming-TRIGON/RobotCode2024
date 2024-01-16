package frc.trigon.robot.subsystems.roller;

import frc.trigon.robot.subsystems.MotorSubsystem;
import org.littletonrobotics.junction.Logger;

public class Roller extends MotorSubsystem {
    private final static Roller INSTANCE = new Roller();
    private final RollerIO rollerIO = RollerIO.generateIO();
    private final RollerInputsAutoLogged rollerInputs = new RollerInputsAutoLogged();
    private RollerConstants.RollerState targetState = RollerConstants.RollerState.STOPPED;

    public static Roller getInstance() {
        return INSTANCE;
    }

    private Roller() {
    }

    @Override
    public void stop() {
        rollerIO.stopMotor();
    }

    @Override
    public void periodic() {
        rollerIO.updateInputs(rollerInputs);
        Logger.processInputs("Roller", rollerInputs);
        if (rollerInputs.infraredSensorTriggered && isCollectionState()) {
            setTargetState(RollerConstants.RollerState.STOPPED);
        }
        updateMechanism();
    }

    private void setTargetVelocity(double targetVelocity) {
        rollerIO.setTargetVelocity(targetVelocity);
        RollerConstants.ROLLER_MECHANISM.setTargetVelocity(targetVelocity);
    }

    void setTargetState(RollerConstants.RollerState state) {
        this.targetState = state;
        setTargetVelocity(state.velocityRevolutionsPerSecond);
    }

    private boolean isCollectionState() {
        return this.targetState == RollerConstants.RollerState.COLLECTION;
    }

    private void updateMechanism() {
        RollerConstants.ROLLER_MECHANISM.updateMechanism(rollerInputs.motorVelocityRotationsPerSecond, targetState.velocityRevolutionsPerSecond);
    }
}

