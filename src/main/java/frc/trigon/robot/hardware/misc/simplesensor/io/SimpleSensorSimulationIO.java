package frc.trigon.robot.hardware.misc.simplesensor.io;

import frc.trigon.robot.hardware.misc.simplesensor.SimpleSensorIO;
import frc.trigon.robot.hardware.misc.simplesensor.SimpleSensorInputsAutoLogged;

import java.util.function.DoubleSupplier;

public class SimpleSensorSimulationIO extends SimpleSensorIO {
    private DoubleSupplier valueSupplier = null;

    @Override
    protected void setSimulationSupplier(DoubleSupplier valueInSimulation) {
        this.valueSupplier = valueInSimulation;
    }

    @Override
    public void updateInputs(SimpleSensorInputsAutoLogged inputs) {
        if (valueSupplier == null)
            return;
        inputs.value = valueSupplier.getAsDouble();
    }
}
