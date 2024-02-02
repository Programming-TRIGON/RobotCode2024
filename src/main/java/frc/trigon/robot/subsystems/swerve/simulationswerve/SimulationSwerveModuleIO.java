package frc.trigon.robot.subsystems.swerve.simulationswerve;

import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import edu.wpi.first.math.geometry.Rotation2d;
import frc.trigon.robot.simulation.SimpleMotorSimulation;
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
        inputs.steerAngleDegrees = Conversions.revolutionsToDegrees(steerMotor.getPositionRevolutions());
        inputs.odometryUpdatesSteerAngleDegrees = new double[]{inputs.steerAngleDegrees};
        inputs.steerVoltage = steerMotor.getVoltage();

        inputs.driveDistanceMeters = Conversions.revolutionsToDistance(driveMotor.getPositionRevolutions(), SimulationSwerveModuleConstants.WHEEL_DIAMETER_METERS);
        inputs.odometryUpdatesDriveDistanceMeters = new double[]{inputs.driveDistanceMeters};
        inputs.driveVelocityMetersPerSecond = Conversions.revolutionsToDistance(driveMotor.getVelocityRevolutionsPerSecond(), SimulationSwerveModuleConstants.WHEEL_DIAMETER_METERS);
        inputs.driveCurrent = driveMotor.getCurrent();
        inputs.driveVoltage = driveMotor.getVoltage();
    }

    @Override
    protected void setTargetOpenLoopVelocity(double targetVelocityMetersPerSecond) {
        final double voltage = velocityToOpenLoopVoltage(
                targetVelocityMetersPerSecond,
                SimulationSwerveModuleConstants.WHEEL_DIAMETER_METERS,
                steerMotor.getVelocityRevolutionsPerSecond(),
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
