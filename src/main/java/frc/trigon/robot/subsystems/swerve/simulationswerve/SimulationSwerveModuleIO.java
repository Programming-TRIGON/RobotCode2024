package frc.trigon.robot.subsystems.swerve.simulationswerve;

import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import edu.wpi.first.math.geometry.Rotation2d;
import frc.trigon.robot.motorsimulation.SimpleMotorSimulation;
import frc.trigon.robot.subsystems.swerve.SwerveModuleIO;
import frc.trigon.robot.subsystems.swerve.SwerveModuleInputsAutoLogged;
import frc.trigon.robot.utilities.Conversions;
import org.littletonrobotics.junction.Logger;

public class SimulationSwerveModuleIO extends SwerveModuleIO {
    private final SimpleMotorSimulation driveMotor, steerMotor;
    private final PositionVoltage steerPositionRequest = new PositionVoltage(0);
    private final VoltageOut driveVoltageRequest = new VoltageOut(0);

    SimulationSwerveModuleIO(SimulationSwerveModuleConstants moduleConstants, String moduleName) {
        super(moduleName);

        driveMotor = moduleConstants.driveMotor;
        steerMotor = moduleConstants.steerMotor;
    }

    @Override
    protected void updateInputs(SwerveModuleInputsAutoLogged inputs) {
        inputs.steerAngleDegrees = Conversions.revolutionsToDegrees(steerMotor.getPosition());

        inputs.driveDistanceMeters = Conversions.revolutionsToDistance(driveMotor.getPosition(), SimulationSwerveModuleConstants.WHEEL_DIAMETER_METERS);
        inputs.driveVelocityMetersPerSecond = Conversions.revolutionsToDistance(driveMotor.getVelocity(), SimulationSwerveModuleConstants.WHEEL_DIAMETER_METERS);
        inputs.driveCurrent = driveMotor.getCurrent();
    }

    @Override
    protected void setTargetOpenLoopVelocity(double targetVelocityMetersPerSecond) {
        final double voltage = velocityToOpenLoopVoltage(
                targetVelocityMetersPerSecond,
                SimulationSwerveModuleConstants.WHEEL_DIAMETER_METERS,
                steerMotor.getVelocity(),
                0,
                SimulationSwerveModuleConstants.MAX_SPEED_REVOLUTIONS_PER_SECOND,
                SimulationSwerveModuleConstants.VOLTAGE_COMPENSATION_SATURATION
        );
        Logger.recordOutput(getLoggingPath() + "driveVoltage", driveMotor.getVoltage());
        driveMotor.setControl(driveVoltageRequest.withOutput(voltage));
    }

    @Override
    protected void setTargetClosedLoopVelocity(double targetVelocityMetersPerSecond) {
        setTargetOpenLoopVelocity(targetVelocityMetersPerSecond);
    }

    @Override
    protected void setTargetAngle(Rotation2d angle) {
        steerMotor.setControl(steerPositionRequest.withPosition(angle.getRotations()));
    }

    @Override
    protected void stop() {
        driveMotor.stop();
        steerMotor.stop();
    }
}
