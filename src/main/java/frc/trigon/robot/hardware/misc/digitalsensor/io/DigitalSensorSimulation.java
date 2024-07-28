package frc.trigon.robot.hardware.misc.digitalsensor.io;

import frc.trigon.robot.hardware.misc.digitalsensor.DigitalSensorIO;
import frc.trigon.robot.hardware.misc.digitalsensor.DigitalSensorInputsAutoLogged;

import java.util.function.BooleanSupplier;

public class DigitalSensorSimulation extends DigitalSensorIO {
    private BooleanSupplier isTriggered = null;

    public DigitalSensorSimulation() {
    }

    @Override
    protected void setSimulationSupplier(BooleanSupplier isTriggeredInSimulation) {
        this.isTriggered = isTriggeredInSimulation;
    }

    public void updateInputs(DigitalSensorInputsAutoLogged inputs) {
        inputs.isTriggered = isTriggered.getAsBoolean();
    }
}
