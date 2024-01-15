package frc.trigon.robot.subsystems.roller.placeholderroller;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.controls.VelocityTorqueCurrentFOC;
import com.ctre.phoenix6.hardware.TalonFX;
import edu.wpi.first.wpilibj.DigitalInput;
import frc.trigon.robot.subsystems.roller.RollerIO;
import frc.trigon.robot.subsystems.roller.RollerInputsAutoLogged;

public class PLACEHOLDERRollerIO extends RollerIO {
    private final TalonFX motor = PLACEHOLDERRollerConstants.MOTOR;
    private final DigitalInput infraredSensor = PLACEHOLDERRollerConstants.INFRARED_SENSOR;
    private final VelocityTorqueCurrentFOC velocityRequest = new VelocityTorqueCurrentFOC(0);

    @Override
    protected void updateInputs(RollerInputsAutoLogged inputs) {
        BaseStatusSignal.refreshAll(
                PLACEHOLDERRollerConstants.VOLTAGE_STATUS_SIGNAL,
                PLACEHOLDERRollerConstants.CURRENT_STATUS_SIGNAL,
                PLACEHOLDERRollerConstants.VELOCITY_STATUS_SIGNAL
        );
        inputs.motorVoltage = PLACEHOLDERRollerConstants.VOLTAGE_STATUS_SIGNAL.getValue();
        inputs.motorCurrent = PLACEHOLDERRollerConstants.CURRENT_STATUS_SIGNAL.getValue();
        inputs.motorVelocity = PLACEHOLDERRollerConstants.VELOCITY_STATUS_SIGNAL.getValue();

        inputs.infraredSensorTriggered = isInfraredSensorTriggered();
    }

    @Override
    protected void setTargetVelocity(double velocity) {
        motor.setControl(velocityRequest.withVelocity(velocity));
    }

    @Override
    protected void stopMotor() {
        motor.stopMotor();
    }

    private boolean isInfraredSensorTriggered() {
        return infraredSensor.get();
    }
}
