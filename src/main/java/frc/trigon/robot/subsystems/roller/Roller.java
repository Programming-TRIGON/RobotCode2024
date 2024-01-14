package frc.trigon.robot.subsystems.roller;


import edu.wpi.first.wpilibj2.command.SubsystemBase;
import org.littletonrobotics.junction.Logger;

public class Roller extends SubsystemBase {
    private final static Roller INSTANCE = new Roller();
    private final RollerIO rollerIO = RollerIO.generateIO();
    private final RollerInputsAutoLogged rollerInputs = new RollerInputsAutoLogged();

    public static Roller getInstance() {
        return INSTANCE;
    }

    private Roller() {
    }

    @Override
    public void periodic() {
        rollerIO.updateInputs(rollerInputs);
        Logger.processInputs("Roller", rollerInputs);
    }

    void setTargetState(RollerConstants.RollerState targetState) {
        rollerIO.setTargetVelocityState(targetState);
    }
    void stopMotor() {
        rollerIO.stopMotor();
    }
}

