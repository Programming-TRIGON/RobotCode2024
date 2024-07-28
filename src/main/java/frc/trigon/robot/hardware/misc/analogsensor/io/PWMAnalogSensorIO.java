package frc.trigon.robot.hardware.misc.analogsensor.io;

import edu.wpi.first.wpilibj.AnalogInput;
import frc.trigon.robot.hardware.misc.analogsensor.AnalogSensorIO;
import frc.trigon.robot.hardware.misc.analogsensor.AnalogSensorInputsAutoLogged;

public class PWMAnalogSensorIO extends AnalogSensorIO {
    private final AnalogInput analogInput;

    public PWMAnalogSensorIO(int channel) {
        analogInput = new AnalogInput(channel);
    }

    public void updateInputs(AnalogSensorInputsAutoLogged inputs) {
        inputs.value = analogInput.getValue();
    }
}
