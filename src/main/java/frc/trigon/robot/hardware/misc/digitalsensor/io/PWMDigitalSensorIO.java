package frc.trigon.robot.hardware.misc.digitalsensor.io;

import edu.wpi.first.wpilibj.DigitalInput;
import frc.trigon.robot.hardware.misc.digitalsensor.DigitalSensorIO;
import frc.trigon.robot.hardware.misc.digitalsensor.DigitalSensorInputsAutoLogged;

public class PWMDigitalSensorIO extends DigitalSensorIO {
    private final DigitalInput digitalInput;

    public PWMDigitalSensorIO(int channel) {
        digitalInput = new DigitalInput(channel);
    }

    public void updateInputs(DigitalSensorInputsAutoLogged inputs) {
        inputs.isTriggered = digitalInput.get();
    }
}
