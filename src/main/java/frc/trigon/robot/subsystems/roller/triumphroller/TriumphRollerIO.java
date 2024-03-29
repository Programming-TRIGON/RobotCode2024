package frc.trigon.robot.subsystems.roller.triumphroller;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.controls.VelocityTorqueCurrentFOC;
import com.ctre.phoenix6.hardware.TalonFX;
import edu.wpi.first.wpilibj.DigitalInput;
import frc.trigon.robot.subsystems.roller.RollerIO;
import frc.trigon.robot.subsystems.roller.RollerInputsAutoLogged;

public class TriumphRollerIO extends RollerIO {
    private final TalonFX motor = TriumphRollerConstants.MOTOR;
    private final DigitalInput infraredSensor = TriumphRollerConstants.INFRARED_SENSOR;
    private final VelocityTorqueCurrentFOC velocityRequest = new VelocityTorqueCurrentFOC(0);

    @Override
    protected void updateInputs(RollerInputsAutoLogged inputs) {
        refreshStatusSignals();
        inputs.motorVoltage = TriumphRollerConstants.VOLTAGE_STATUS_SIGNAL.getValue();
        inputs.motorCurrent = TriumphRollerConstants.CURRENT_STATUS_SIGNAL.getValue();
        inputs.motorVelocityRevolutionsPerSecond = TriumphRollerConstants.VELOCITY_STATUS_SIGNAL.getValue();

        inputs.infraredSensorTriggered = isInfraredSensorTriggered();
    }

    @Override
    protected void setTargetVelocity(double targetVelocityRevolutionsPerSecond) {
        motor.setControl(velocityRequest.withVelocity(targetVelocityRevolutionsPerSecond));
    }

    @Override
    protected void stopMotor() {
        motor.stopMotor();
    }

    private boolean isInfraredSensorTriggered() {
        return infraredSensor.get();
    }

    private void refreshStatusSignals() {
        BaseStatusSignal.refreshAll(
                TriumphRollerConstants.VOLTAGE_STATUS_SIGNAL,
                TriumphRollerConstants.CURRENT_STATUS_SIGNAL,
                TriumphRollerConstants.VELOCITY_STATUS_SIGNAL
        );
    }
}
