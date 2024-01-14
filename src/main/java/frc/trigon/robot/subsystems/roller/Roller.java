package frc.trigon.robot.subsystems.roller;


import frc.trigon.robot.subsystems.MotorSubsystem;
import org.littletonrobotics.junction.Logger;

public class Roller extends MotorSubsystem {
    private final static Roller INSTANCE = new Roller();
    private final RollerIO rollerIO = RollerIO.generateIO();
    private final RollerInputsAutoLogged rollerInputs = new RollerInputsAutoLogged();

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
        if (isInfraredSensorTriggered()) {
            stop();
            setTargetState(RollerConstants.RollerState.DEFAULT);
        }
    }

    void setTargetVelocity(double velocity) {
        rollerIO.setTargetVelocityState(velocity);
    }

    void setTargetState(RollerConstants.RollerState state) {
        setTargetVelocity(state.velocityRevolutionsPerSecond);
    }

    private boolean isInfraredSensorTriggered() {
        return rollerInputs.infraredSensorTriggered;
    }
}

