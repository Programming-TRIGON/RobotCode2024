package frc.trigon.robot.subsystems.swerve.trihardswerve;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.controls.VelocityTorqueCurrentFOC;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.math.geometry.Rotation2d;
import frc.trigon.robot.poseestimation.poseestimator.TalonFXOdometryThread;
import frc.trigon.robot.subsystems.swerve.SwerveModuleIO;
import frc.trigon.robot.subsystems.swerve.SwerveModuleInputsAutoLogged;
import frc.trigon.robot.utilities.Conversions;

import java.util.Queue;

public class TrihardSwerveModuleIO extends SwerveModuleIO {
    private final TalonFX steerMotor, driveMotor;
    private final TrihardSwerveModuleConstants moduleConstants;
    private final Queue<Double> steerPositionQueue, drivePositionQueue;

    private final VelocityTorqueCurrentFOC driveVelocityRequest = new VelocityTorqueCurrentFOC(0);
    private final VoltageOut driveVoltageRequest = new VoltageOut(0).withEnableFOC(TrihardSwerveModuleConstants.ENABLE_FOC);
    private final PositionVoltage steerPositionRequest = new PositionVoltage(0).withEnableFOC(TrihardSwerveModuleConstants.ENABLE_FOC);

    TrihardSwerveModuleIO(TrihardSwerveModuleConstants moduleConstants, String moduleName) {
        super(moduleName);

        this.steerMotor = moduleConstants.steerMotor;
        this.driveMotor = moduleConstants.driveMotor;
        this.moduleConstants = moduleConstants;
        steerPositionQueue = TalonFXOdometryThread.getInstance().registerSignal(steerMotor, moduleConstants.steerPositionSignal);
        drivePositionQueue = TalonFXOdometryThread.getInstance().registerSignal(driveMotor, moduleConstants.drivePositionSignal);
    }

    @Override
    protected void updateInputs(SwerveModuleInputsAutoLogged inputs) {
        refreshStatusSignals();

        inputs.steerAngleDegrees = getAngleDegrees();
        inputs.odometrySteerAnglesDegrees = steerPositionQueue.stream().mapToDouble(Conversions::revolutionsToDegrees).toArray();

        inputs.driveDistanceMeters = toDriveDistance(moduleConstants.drivePositionSignal.getValue());
        inputs.odometryDriveDistancesMeters = drivePositionQueue.stream().mapToDouble(this::toDriveDistance).toArray();
        inputs.driveVelocityMetersPerSecond = toDriveDistance(moduleConstants.driveVelocitySignal.getValue());
        inputs.driveCurrent = moduleConstants.driveStatorCurrentSignal.getValue();

        steerPositionQueue.clear();
        drivePositionQueue.clear();
    }

    @Override
    protected void setTargetOpenLoopVelocity(double targetVelocityMetersPerSecond) {
        final double voltage = velocityToOpenLoopVoltage(
                targetVelocityMetersPerSecond,
                TrihardSwerveModuleConstants.WHEEL_DIAMETER_METERS,
                moduleConstants.steerVelocitySignal.getValue(),
                TrihardSwerveModuleConstants.COUPLING_RATIO,
                TrihardSwerveModuleConstants.MAX_SPEED_REVOLUTIONS_PER_SECOND,
                TrihardSwerveModuleConstants.VOLTAGE_COMPENSATION_SATURATION
        );
        driveMotor.setControl(driveVoltageRequest.withOutput(voltage));
    }

    @Override
    protected void setTargetClosedLoopVelocity(double targetVelocityMetersPerSecond) {
        final double optimizedVelocityRevolutionsPerSecond = removeCouplingFromRevolutions(
                targetVelocityMetersPerSecond,
                Rotation2d.fromDegrees(moduleConstants.steerVelocitySignal.getValue()),
                TrihardSwerveModuleConstants.COUPLING_RATIO
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
        return Conversions.revolutionsToDistance(revolutions, TrihardSwerveModuleConstants.WHEEL_DIAMETER_METERS);
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
