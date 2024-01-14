package frc.trigon.robot.subsystems.roller.placeholderroller;

import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.controls.VelocityTorqueCurrentFOC;
import com.ctre.phoenix6.hardware.TalonFX;
import edu.wpi.first.wpilibj.DigitalInput;
import frc.trigon.robot.subsystems.roller.RollerConstants;
import frc.trigon.robot.subsystems.roller.RollerIO;
import frc.trigon.robot.subsystems.roller.RollerInputsAutoLogged;

public class PLACEHOLDERRollerIO extends RollerIO {
    private final TalonFX motor = PLACEHOLDERRollerConstants.MOTOR;
    private final DigitalInput infraredSensor = PLACEHOLDERRollerConstants.INFRARED_SENSOR;
    private final VelocityTorqueCurrentFOC currentRequest = new VelocityTorqueCurrentFOC(0);
    private final StatusSignal<Double>
            VOLTAGE_STATUS_SIGNAL = motor.getMotorVoltage(),
            CURRENT_STATUS_SIGNAL = motor.getTorqueCurrent();

    @Override
    protected void updateInputs(RollerInputsAutoLogged inputs) {
        inputs.motorVoltage = VOLTAGE_STATUS_SIGNAL.refresh().getValue();
        inputs.motorCurrent = CURRENT_STATUS_SIGNAL.refresh().getValue();

        inputs.infraredSensorTriggered = isInfraredSensorTriggered();
    }

    @Override
    protected void setTargetVelocityState(RollerConstants.RollerState targetState) {
        motor.setControl(currentRequest.withVelocity(targetState.current));
    }

    @Override
    protected void stopMotor() {
        motor.stopMotor();
    }

    private boolean isInfraredSensorTriggered() {
        return infraredSensor.get();
    }
}
