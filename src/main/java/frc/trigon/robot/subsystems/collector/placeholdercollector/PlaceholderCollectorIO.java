package frc.trigon.robot.subsystems.collector.placeholdercollector;

import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import frc.trigon.robot.subsystems.collector.CollectorIO;

public class PlaceholderCollectorIO extends CollectorIO {
    private final TalonFX
            collectionMotor = PlaceholderCollectorConstants.COLLECTING_MOTOR,
            angleMotor = PlaceholderCollectorConstants.ANGLE_MOTOR;
    private final VoltageOut voltageRequest = new VoltageOut(0).withEnableFOC(PlaceholderCollectorConstants.FOC_ENABLED);

    @Override

    protected void updateInputs() {
        
    }

    @Override
    protected void setVoltage(double voltage) {
        super.setVoltage(voltage);
    }

    @Override
    protected void stop() {
        super.stop();
    }
}
