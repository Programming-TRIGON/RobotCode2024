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
    }

    void setTargetState(RollerConstants.RollerState targetState) {
        rollerIO.setTargetVelocityState(targetState);
    }
}

