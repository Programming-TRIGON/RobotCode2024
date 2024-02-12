package frc.trigon.robot.subsystems.swerve.trihardswerve;

import com.revrobotics.CANSparkBase;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import edu.wpi.first.math.geometry.Rotation2d;
import frc.trigon.robot.poseestimation.poseestimator.SparkMaxOdometryThread6328;
import frc.trigon.robot.subsystems.swerve.SwerveModuleIO;
import frc.trigon.robot.subsystems.swerve.SwerveModuleInputsAutoLogged;
import frc.trigon.robot.utilities.Conversions;

import java.util.Queue;

public class TrihardSwerveModuleIO extends SwerveModuleIO {
    private final CANSparkMax steerMotor, driveMotor;
    private final RelativeEncoder steerEncoder, driveEncoder;
    private final Queue<Double> steerPositionQueue, drivePositionQueue;


    TrihardSwerveModuleIO(TrihardSwerveModuleConstants moduleConstants, String moduleName) {
        super(moduleName);

        this.steerMotor = moduleConstants.steerMotor;
        this.driveMotor = moduleConstants.driveMotor;
        steerEncoder = moduleConstants.steerMotor.getEncoder();
        driveEncoder = moduleConstants.driveMotor.getEncoder();
        steerPositionQueue = SparkMaxOdometryThread6328.getInstance().registerSignal(steerEncoder::getPosition);
        drivePositionQueue = SparkMaxOdometryThread6328.getInstance().registerSignal(driveEncoder::getPosition);
    }

    @Override
    protected void updateInputs(SwerveModuleInputsAutoLogged inputs) {
        inputs.steerAngleDegrees = getAngleDegrees();
        inputs.odometryUpdatesSteerAngleDegrees = steerPositionQueue.stream().mapToDouble(Conversions::revolutionsToDegrees).toArray();
        inputs.steerVoltage = steerMotor.getBusVoltage() * steerMotor.getAppliedOutput();

        inputs.driveDistanceMeters = toDriveDistance(driveEncoder.getPosition());
        inputs.odometryUpdatesDriveDistanceMeters = drivePositionQueue.stream().mapToDouble(this::toDriveDistance).toArray();
        inputs.driveVelocityMetersPerSecond = toDriveDistance(driveEncoder.getVelocity());
        inputs.driveCurrent = driveMotor.getOutputCurrent();
        inputs.driveVoltage = driveMotor.getBusVoltage() * driveMotor.getAppliedOutput();

        steerPositionQueue.clear();
        drivePositionQueue.clear();
    }

    @Override
    protected void setTargetOpenLoopVelocity(double targetVelocityMetersPerSecond) {
        final double voltage = velocityToOpenLoopVoltage(
                targetVelocityMetersPerSecond,
                TrihardSwerveModuleConstants.WHEEL_DIAMETER_METERS,
                steerEncoder.getPosition(),
                TrihardSwerveModuleConstants.COUPLING_RATIO,
                TrihardSwerveModuleConstants.MAX_SPEED_REVOLUTIONS_PER_SECOND,
                TrihardSwerveModuleConstants.VOLTAGE_COMPENSATION_SATURATION
        );
        driveMotor.getPIDController().setReference(voltage, CANSparkBase.ControlType.kVoltage);
    }

    @Override
    protected void setTargetClosedLoopVelocity(double targetVelocityMetersPerSecond) {
        final double optimizedVelocityRevolutionsPerSecond = removeCouplingFromRevolutions(
                targetVelocityMetersPerSecond,
                Rotation2d.fromRotations(steerEncoder.getPosition()),
                TrihardSwerveModuleConstants.COUPLING_RATIO
        );
        driveMotor.getPIDController().setReference(optimizedVelocityRevolutionsPerSecond, CANSparkBase.ControlType.kVelocity);
    }

    @Override
    protected void setTargetAngle(Rotation2d angle) {
        steerMotor.getPIDController().setReference(angle.getRotations(), CANSparkBase.ControlType.kPosition);
    }

    @Override
    protected void stop() {
        steerMotor.stopMotor();
        driveMotor.stopMotor();
    }

    @Override
    protected void setBrake(boolean brake) {
        final CANSparkBase.IdleMode idleMode = brake ? CANSparkBase.IdleMode.kBrake : CANSparkBase.IdleMode.kCoast;
        driveMotor.setIdleMode(idleMode);
        steerMotor.setIdleMode(idleMode);
    }

    private double getAngleDegrees() {
        return Conversions.revolutionsToDegrees(steerEncoder.getPosition());
    }

    private double toDriveDistance(double revolutions) {
        return Conversions.revolutionsToDistance(revolutions, TrihardSwerveModuleConstants.WHEEL_DIAMETER_METERS);
    }
}
