package frc.trigon.robot.subsystems.roller;

import frc.trigon.robot.subsystems.MotorSubsystem;
import org.littletonrobotics.junction.Logger;

public class Roller extends MotorSubsystem {
    private final static Roller INSTANCE = new Roller();
    private final RollerIO rollerIO = RollerIO.generateIO();
    private final RollerInputsAutoLogged rollerInputs = new RollerInputsAutoLogged();
    private double velocity;
    private RollerConstants.RollerState targetState;

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
        if (isInfraredSensorTriggered() && isCollectionState()) {
            stop();
            setTargetState(RollerConstants.RollerState.DEFAULT);
        }
        updateMechanism();
    }

    void setTargetVelocity(double velocity) {
        this.velocity = velocity;
        rollerIO.setTargetVelocityRotationsPerSecond(velocity);
        RollerConstants.ROLLER_MECHANISM.setTargetVelocity(velocity);
    }

    void setTargetState(RollerConstants.RollerState state) {
        this.targetState = state;
        this.velocity = state.velocityRevolutionsPerSecond;
        setTargetVelocity(velocity);
        RollerConstants.ROLLER_MECHANISM.setTargetVelocity(velocity);
    }

    private boolean isInfraredSensorTriggered() {
        return rollerInputs.infraredSensorTriggered;
    }

    private boolean isCollectionState() {
        return this.targetState == RollerConstants.RollerState.COLLECTION;
    }

    private void updateMechanism() {
        RollerConstants.ROLLER_MECHANISM.updateMechanism(velocity);
    }
}

