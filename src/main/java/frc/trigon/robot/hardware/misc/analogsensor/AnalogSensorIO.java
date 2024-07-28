package frc.trigon.robot.hardware.misc.analogsensor;

import org.littletonrobotics.junction.AutoLog;

import java.util.function.DoubleSupplier;

public class AnalogSensorIO {
    protected AnalogSensorIO() {
    }

    protected void setSimulationSupplier(DoubleSupplier valueInSimulation) {
    }

    protected void updateInputs(AnalogSensorInputsAutoLogged inputs) {
    }

    @AutoLog
    protected static class AnalogSensorInputs {
        public double value;
    }
}
