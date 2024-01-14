package frc.trigon.robot.subsystems.roller.placeholderroller;

import com.ctre.phoenix6.controls.VelocityTorqueCurrentFOC;
import com.ctre.phoenix6.hardware.TalonFX;
import edu.wpi.first.wpilibj.DigitalInput;
import frc.trigon.robot.subsystems.roller.RollerIO;
import frc.trigon.robot.subsystems.roller.RollerInputsAutoLogged;

public class PLACEHOLDERRollerIO extends RollerIO {
    private final TalonFX motor = PLACEHOLDERRollerConstants.MOTOR;
    private final DigitalInput infraredSensor = PLACEHOLDERRollerConstants.INFRARED_SENSOR;
    private final VelocityTorqueCurrentFOC currentRequest = new VelocityTorqueCurrentFOC(0);

    @Override
    protected void updateInputs(RollerInputsAutoLogged inputs) {
        inputs.motorVoltage = getVoltage();
        inputs.motorCurrent = getCurrent();
        inputs.motorCurrentVelocity = getCurrentVelocityRotationsPerSecond();

        inputs.infraredSensorTriggered = isInfraredSensorTriggered();
    }

    @Override
    protected void setTargetVelocityState(double velocity) {
        motor.setControl(currentRequest.withVelocity(velocity));
    }

    @Override
    protected void stopMotor() {
        motor.stopMotor();
    }

    private boolean isInfraredSensorTriggered() {
        return infraredSensor.get();
    }

    private double getVoltage() {
        return PLACEHOLDERRollerConstants.VOLTAGE_STATUS_SIGNAL.refresh().getValue();
    }

    private double getCurrent() {
        return PLACEHOLDERRollerConstants.CURRENT_STATUS_SIGNAL.refresh().getValue();
    }

    private double getCurrentVelocityRotationsPerSecond() {
        return PLACEHOLDERRollerConstants.VELOCITY_STATUS_SIGNAL.refresh().getValue();
    }
}
