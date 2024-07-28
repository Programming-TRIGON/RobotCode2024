package frc.trigon.robot.hardware.misc.digitalsensor;

import frc.trigon.robot.constants.RobotConstants;
import frc.trigon.robot.hardware.misc.digitalsensor.io.DigitalSensorSimulation;
import frc.trigon.robot.hardware.misc.digitalsensor.io.PWMDigitalSensorIO;
import org.littletonrobotics.junction.Logger;

import java.util.function.BooleanSupplier;

public class DigitalSensor {
    private final String name;
    private final DigitalSensorIO sensorIO;
    private final DigitalSensorInputsAutoLogged sensorInputs = new DigitalSensorInputsAutoLogged();

    public DigitalSensor(String name, int channel) {
        this.name = name;
        this.sensorIO = generateIO(channel);
    }

    public void setSimulationSupplier(BooleanSupplier isTriggeredInSimulation) {
        sensorIO.setSimulationSupplier(isTriggeredInSimulation);
    }

    public boolean isTriggered() {
        return sensorInputs.isTriggered;
    }

    public void updateSensor() {
        sensorIO.updateInputs(sensorInputs);
        Logger.processInputs("DigitalSensors/" + name, sensorInputs);
    }

    private DigitalSensorIO generateIO(int channel) {
        if (RobotConstants.IS_REPLAY)
            return new DigitalSensorIO();
        if (RobotConstants.IS_SIMULATION)
            return new DigitalSensorSimulation();
        return new PWMDigitalSensorIO(channel);
    }
}
