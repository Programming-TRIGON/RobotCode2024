package frc.trigon.robot.subsystems.swerve.trihardswerve;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.TalonFXConfigurator;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.controls.VelocityTorqueCurrentFOC;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.math.geometry.Rotation2d;
import frc.trigon.robot.subsystems.swerve.SwerveModuleIO;
import frc.trigon.robot.subsystems.swerve.SwerveModuleInputsAutoLogged;
import frc.trigon.robot.utilities.Conversions;

public class TrihardSwerveModuleIO extends SwerveModuleIO {
    private final TalonFX steerMotor, driveMotor;
    private final TrihardSwerveModuleConstants moduleConstants;

    private final VelocityTorqueCurrentFOC driveVelocityRequest = new VelocityTorqueCurrentFOC(0);
    private final VoltageOut driveVoltageRequest = new VoltageOut(0).withEnableFOC(TrihardSwerveModuleConstants.ENABLE_FOC);
    private final PositionVoltage steerPositionRequest = new PositionVoltage(0).withEnableFOC(TrihardSwerveModuleConstants.ENABLE_FOC);

    TrihardSwerveModuleIO(TrihardSwerveModuleConstants moduleConstants, String moduleName) {
        super(moduleName);

        this.steerMotor = moduleConstants.steerMotor;
        this.driveMotor = moduleConstants.driveMotor;
        this.moduleConstants = moduleConstants;
    }

    @Override
    protected void updateInputs(SwerveModuleInputsAutoLogged inputs) {
        inputs.steerAngleDegrees = getAngleDegrees();

        inputs.driveDistanceMeters = getDriveDistance(Rotation2d.fromDegrees(inputs.steerAngleDegrees));
        inputs.driveVelocityMetersPerSecond = Conversions.revolutionsToDistance(moduleConstants.driveVelocitySignal.getValue(), TrihardSwerveModuleConstants.WHEEL_DIAMETER_METERS);
        inputs.driveCurrent = moduleConstants.driveStatorCurrentSignal.refresh().getValue();
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
        setBrake(driveMotor, neutralModeValue);
        setBrake(steerMotor, neutralModeValue);
    }

    private double getAngleDegrees() {
        BaseStatusSignal.refreshAll(moduleConstants.steerPositionSignal, moduleConstants.steerVelocitySignal);
        final double latencyCompensatedRevolutions = BaseStatusSignal.getLatencyCompensatedValue(moduleConstants.steerPositionSignal, moduleConstants.steerVelocitySignal);
        return Conversions.revolutionsToDegrees(latencyCompensatedRevolutions);
    }

    private double getDriveDistance(Rotation2d moduleAngle) {
        BaseStatusSignal.refreshAll(moduleConstants.drivePositionSignal, moduleConstants.driveVelocitySignal);
        final double latencyCompensatedRevolutions = BaseStatusSignal.getLatencyCompensatedValue(moduleConstants.drivePositionSignal, moduleConstants.driveVelocitySignal);
        final double revolutionsWithoutCoupling = removeCouplingFromRevolutions(latencyCompensatedRevolutions, moduleAngle, TrihardSwerveModuleConstants.COUPLING_RATIO);
        return Conversions.revolutionsToDistance(revolutionsWithoutCoupling, TrihardSwerveModuleConstants.WHEEL_DIAMETER_METERS);
    }

    private void setBrake(TalonFX motor, NeutralModeValue neutralModeValue) {
        final MotorOutputConfigs config = new MotorOutputConfigs();
        final TalonFXConfigurator configurator = motor.getConfigurator();

        configurator.refresh(config, 10);
        config.NeutralMode = neutralModeValue;
        configurator.apply(config, 10);
    }
}
