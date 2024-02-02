package frc.trigon.robot.subsystems.swerve.placeholderswere;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.controls.VelocityTorqueCurrentFOC;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.math.geometry.Rotation2d;
import frc.trigon.robot.poseestimation.poseestimator.TalonFXOdometryThread6328;
import frc.trigon.robot.subsystems.swerve.SwerveModuleIO;
import frc.trigon.robot.subsystems.swerve.SwerveModuleInputsAutoLogged;
import frc.trigon.robot.utilities.Conversions;

import java.util.Queue;

public class PLACEHOLDERSwerveModuleIO extends SwerveModuleIO {
    private final TalonFX steerMotor, driveMotor;
    private final PLACEHOLDERSwerveModuleConstants moduleConstants;
    private final Queue<Double> steerPositionQueue, drivePositionQueue;

    private final VelocityTorqueCurrentFOC driveVelocityRequest = new VelocityTorqueCurrentFOC(0);
    private final VoltageOut driveVoltageRequest = new VoltageOut(0).withEnableFOC(PLACEHOLDERSwerveModuleConstants.ENABLE_FOC);
    private final PositionVoltage steerPositionRequest = new PositionVoltage(0).withEnableFOC(PLACEHOLDERSwerveModuleConstants.ENABLE_FOC);

    PLACEHOLDERSwerveModuleIO(PLACEHOLDERSwerveModuleConstants moduleConstants, String moduleName) {
        super(moduleName);

        this.steerMotor = moduleConstants.steerMotor;
        this.driveMotor = moduleConstants.driveMotor;
        this.moduleConstants = moduleConstants;
        steerPositionQueue = TalonFXOdometryThread6328.getInstance().registerSignal(steerMotor, moduleConstants.steerPositionSignal);
        drivePositionQueue = TalonFXOdometryThread6328.getInstance().registerSignal(driveMotor, moduleConstants.drivePositionSignal);
    }

    @Override
    protected void updateInputs(SwerveModuleInputsAutoLogged inputs) {
        refreshStatusSignals();

        inputs.steerAngleDegrees = getAngleDegrees();
        inputs.odometryUpdatesSteerAngleDegrees = steerPositionQueue.stream().mapToDouble(Conversions::revolutionsToDegrees).toArray();
        inputs.steerVoltage = moduleConstants.steerVoltageSignal.getValue();

        inputs.driveDistanceMeters = toDriveDistance(moduleConstants.drivePositionSignal.getValue());
        inputs.odometryUpdatesDriveDistanceMeters = drivePositionQueue.stream().mapToDouble(this::toDriveDistance).toArray();
        inputs.driveVelocityMetersPerSecond = toDriveDistance(moduleConstants.driveVelocitySignal.getValue());
        inputs.driveCurrent = moduleConstants.driveStatorCurrentSignal.getValue();
        inputs.driveVoltage = moduleConstants.driveVoltageSignal.getValue();

        steerPositionQueue.clear();
        drivePositionQueue.clear();
    }

    @Override
    protected void setTargetOpenLoopVelocity(double targetVelocityMetersPerSecond) {
        final double voltage = velocityToOpenLoopVoltage(
                targetVelocityMetersPerSecond,
                PLACEHOLDERSwerveModuleConstants.WHEEL_DIAMETER_METERS,
                moduleConstants.steerVelocitySignal.getValue(),
                PLACEHOLDERSwerveModuleConstants.COUPLING_RATIO,
                PLACEHOLDERSwerveModuleConstants.MAX_SPEED_REVOLUTIONS_PER_SECOND,
                PLACEHOLDERSwerveModuleConstants.VOLTAGE_COMPENSATION_SATURATION
        );
        driveMotor.setControl(driveVoltageRequest.withOutput(voltage));
    }

    @Override
    protected void setTargetClosedLoopVelocity(double targetVelocityMetersPerSecond) {
        final double optimizedVelocityRevolutionsPerSecond = removeCouplingFromRevolutions(
                targetVelocityMetersPerSecond,
                Rotation2d.fromDegrees(moduleConstants.steerVelocitySignal.getValue()),
                PLACEHOLDERSwerveModuleConstants.COUPLING_RATIO
        );
        driveMotor.setControl(driveVelocityRequest.withVelocity(optimizedVelocityRevolutionsPerSecond));
    }

    @Override
    protected void setTargetAngle(Rotation2d angle) {
        steerMotor.setControl(steerPositionRequest.withPosition(angle.getRotations()));
    }

    @Override
    protected void stop() {
        steerMotor.stopMotor();
        driveMotor.stopMotor();
    }

    @Override
    protected void setBrake(boolean brake) {
        final NeutralModeValue neutralModeValue = brake ? NeutralModeValue.Brake : NeutralModeValue.Coast;
        driveMotor.setNeutralMode(neutralModeValue);
        steerMotor.setNeutralMode(neutralModeValue);
    }

    private double getAngleDegrees() {
        return Conversions.revolutionsToDegrees(moduleConstants.steerPositionSignal.getValue());
    }

    private double toDriveDistance(double revolutions) {
        return Conversions.revolutionsToDistance(revolutions, PLACEHOLDERSwerveModuleConstants.WHEEL_DIAMETER_METERS);
    }

    private void refreshStatusSignals() {
        BaseStatusSignal.refreshAll(
                moduleConstants.steerPositionSignal,
                moduleConstants.steerVelocitySignal,
                moduleConstants.drivePositionSignal,
                moduleConstants.driveVelocitySignal,
                moduleConstants.driveStatorCurrentSignal
        );
    }
}
