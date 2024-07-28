package frc.trigon.robot.hardware.misc.analogsensor.io;

import frc.trigon.robot.hardware.misc.analogsensor.AnalogSensorIO;
import frc.trigon.robot.hardware.misc.analogsensor.AnalogSensorInputsAutoLogged;

import java.util.function.DoubleSupplier;

public class AnalogSensorSimulation extends AnalogSensorIO {
    private DoubleSupplier valueSupplier = null;

    @Override
    protected void setSimulationSupplier(DoubleSupplier valueInSimulation) {
        this.valueSupplier = valueInSimulation;
    }

    @Override
    public void updateInputs(AnalogSensorInputsAutoLogged inputs) {
        if (valueSupplier == null)
            return;
        inputs.value = valueSupplier.getAsDouble();
    }
}
