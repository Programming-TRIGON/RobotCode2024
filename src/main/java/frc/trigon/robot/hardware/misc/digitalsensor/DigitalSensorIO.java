package frc.trigon.robot.hardware.misc.digitalsensor;

import org.littletonrobotics.junction.AutoLog;

import java.util.function.BooleanSupplier;

public class DigitalSensorIO {
    protected DigitalSensorIO() {
    }

    protected void updateInputs(DigitalSensorInputsAutoLogged inputs) {
    }

    protected void setSimulationSupplier(BooleanSupplier isTriggeredInSimulation) {
    }

    @AutoLog
    protected static class DigitalSensorInputs {
        public boolean isTriggered;
    }
}
