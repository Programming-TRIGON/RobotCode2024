package frc.trigon.robot.hardware.misc.analogsensor;

import frc.trigon.robot.constants.RobotConstants;
import frc.trigon.robot.hardware.misc.analogsensor.io.AnalogSensorSimulation;
import frc.trigon.robot.hardware.misc.analogsensor.io.PWMAnalogSensorIO;
import org.littletonrobotics.junction.Logger;

import java.util.function.DoubleSupplier;

public class AnalogSensor {
    private final String name;
    private final AnalogSensorIO sensorIO;
    private final AnalogSensorInputsAutoLogged sensorInputs = new AnalogSensorInputsAutoLogged();

    public AnalogSensor(String name, int channel) {
        this.name = name;
        this.sensorIO = generateIO(channel);
    }

    public double getValue() {
        return sensorInputs.value;
    }

    public void setSimulationSupplier(DoubleSupplier valueInSimulation) {
        sensorIO.setSimulationSupplier(valueInSimulation);
    }

    public void updateSensor() {
        sensorIO.updateInputs(sensorInputs);
        Logger.processInputs("AnalogSensors/" + name, sensorInputs);
    }

    private AnalogSensorIO generateIO(int channel) {
        if (RobotConstants.IS_REPLAY)
            return new AnalogSensorIO();
        if (RobotConstants.IS_SIMULATION)
            return new AnalogSensorSimulation();
        return new PWMAnalogSensorIO(channel);
    }
}
