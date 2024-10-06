package frc.trigon.robot.subsystems.elevator;

import com.ctre.phoenix6.controls.DynamicMotionMagicVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.units.Measure;
import edu.wpi.first.units.Units;
import edu.wpi.first.units.Voltage;
import edu.wpi.first.wpilibj.sysid.SysIdRoutineLog;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.trigon.robot.subsystems.MotorSubsystem;
import org.littletonrobotics.junction.Logger;
import org.trigon.hardware.phoenix6.cancoder.CANcoderSignal;
import org.trigon.hardware.phoenix6.talonfx.TalonFXMotor;
import org.trigon.hardware.phoenix6.talonfx.TalonFXSignal;
import org.trigon.utilities.Conversions;

public class Elevator extends MotorSubsystem {
    private final TalonFXMotor motor = ElevatorConstants.MASTER_MOTOR;
    private final DynamicMotionMagicVoltage positionRequest = new DynamicMotionMagicVoltage(
            0,
            ElevatorConstants.MOTION_MAGIC_CRUISE_VELOCITY,
            ElevatorConstants.MOTION_MAGIC_ACCELERATION,
            ElevatorConstants.MOTION_MAGIC_JERK
    ).withUpdateFreqHz(1000).withEnableFOC(ElevatorConstants.FOC_ENABLED);
    private final VoltageOut voltageRequest = new VoltageOut(0).withEnableFOC(ElevatorConstants.FOC_ENABLED).withUpdateFreqHz(1000);
    private ElevatorConstants.ElevatorState targetState = ElevatorConstants.ElevatorState.RESTING;
    private boolean didOpenElevator = false;

    public Elevator() {
        setName("Elevator");
    }

    @Override
    public void updatePeriodically() {
        motor.update();
        ElevatorConstants.ENCODER.update();
        updateNetworkTables();
    }

    @Override
    public void updateMechanism() {
        Logger.recordOutput("Poses/Components/ElevatorPose", getElevatorComponentPose());
        Logger.recordOutput("Poses/Components/TransporterPose", getTransporterComponentPose());
        ElevatorConstants.MECHANISM.update(getPositionMeters(), toMeters(motor.getSignal(TalonFXSignal.CLOSED_LOOP_REFERENCE)));
    }

    @Override
    public void updateLog(SysIdRoutineLog log) {
        log.motor("Elevator")
                .angularPosition(Units.Rotations.of(getEncoderPosition()))
                .angularVelocity(Units.RotationsPerSecond.of(motor.getSignal(TalonFXSignal.ROTOR_VELOCITY) / ElevatorConstants.GEAR_RATIO))
                .voltage(Units.Volts.of(motor.getSignal(TalonFXSignal.MOTOR_VOLTAGE)));
    }

    @Override
    public SysIdRoutine.Config getSysIdConfig() {
        return ElevatorConstants.SYSID_CONFIG;
    }

    @Override
    public void setBrake(boolean brake) {
        motor.setBrake(brake);
        ElevatorConstants.FOLLOWER_MOTOR.setBrake(brake);
    }

    @Override
    public void stop() {
        motor.stopMotor();
    }

    @Override
    public void drive(Measure<Voltage> voltageMeasure) {
        motor.setControl(voltageRequest.withOutput(voltageMeasure.in(Units.Volts)));
    }

    public double getRotorPosition() {
        return motor.getSignal(TalonFXSignal.ROTOR_POSITION);
    }

    public double getEncoderPosition() {
        return ElevatorConstants.ENCODER.getSignal(CANcoderSignal.POSITION);
    }

    public boolean atTargetState() {
        return Math.abs(this.targetState.positionMeters - getPositionMeters()) < ElevatorConstants.TOLERANCE_METERS;
    }

    public boolean isOpenForTrap() {
        return getPositionMeters() > 0.32;
    }

    public ElevatorConstants.ElevatorState getTargetState() {
        return targetState;
    }

    public boolean isResting() {
        return targetState == ElevatorConstants.ElevatorState.RESTING;
    }

    public boolean didOpenElevator() {
        return didOpenElevator;
    }

    public void setDidOpenElevator(boolean didOpenElevator) {
        this.didOpenElevator = didOpenElevator;
    }

    public boolean isBelowCameraPlate() {
        return getPositionMeters() < ElevatorConstants.CAMERA_PLATE_HEIGHT_METERS;
    }

    void setTargetState(ElevatorConstants.ElevatorState targetState) {
        this.targetState = targetState;
        setTargetPosition(targetState.positionMeters, targetState.speedPercentage);
    }

    void setTargetPosition(double targetPositionMeters, double speedPercentage) {
        motor.setControl(scaleProfile(positionRequest.withPosition(toRotations(targetPositionMeters)), speedPercentage));
    }

    private void updateNetworkTables() {
        Logger.recordOutput("Elevator/ElevatorPositionMeters", getPositionMeters());
        Logger.recordOutput("Elevator/ElevatorVelocityMetersPerSecond", toMeters(motor.getSignal(TalonFXSignal.VELOCITY)));
    }

    private DynamicMotionMagicVoltage scaleProfile(DynamicMotionMagicVoltage profile, double speedPercentage) {
        return profile.withVelocity(ElevatorConstants.MOTION_MAGIC_CRUISE_VELOCITY * (speedPercentage / 100)).withAcceleration(ElevatorConstants.MOTION_MAGIC_ACCELERATION * (speedPercentage / 100));
    }

    private Pose3d getElevatorComponentPose() {
        final Transform3d elevatorTransform = new Transform3d(
                new Translation3d(0, 0, getPositionMeters()),
                new Rotation3d()
        );
        return ElevatorConstants.ELEVATOR_ORIGIN_POINT.transformBy(elevatorTransform);
    }

    public Pose3d getTransporterComponentPose() {
        final Transform3d transporterTransform = new Transform3d(
                new Translation3d(0, 0, getPositionMeters() * 2),
                new Rotation3d()
        );
        return ElevatorConstants.TRANSPORTER_ORIGIN_POINT.transformBy(transporterTransform);
    }

    private double getPositionMeters() {
        return toMeters(motor.getSignal(TalonFXSignal.POSITION));
    }

    private double toMeters(double rotations) {
        return Conversions.rotationsToDistance(rotations, ElevatorConstants.DRUM_DIAMETER_METERS);
    }

    private double toRotations(double meters) {
        return Conversions.distanceToRotations(meters, ElevatorConstants.DRUM_DIAMETER_METERS);
    }
}