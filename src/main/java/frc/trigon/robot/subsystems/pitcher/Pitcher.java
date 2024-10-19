package frc.trigon.robot.subsystems.pitcher;

import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import edu.wpi.first.math.geometry.*;
import edu.wpi.first.units.Measure;
import edu.wpi.first.units.Units;
import edu.wpi.first.units.Voltage;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.sysid.SysIdRoutineLog;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.trigon.robot.misc.ShootingCalculations;
import frc.trigon.robot.subsystems.MotorSubsystem;
import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;
import org.trigon.hardware.phoenix6.cancoder.CANcoderSignal;
import org.trigon.hardware.phoenix6.talonfx.TalonFXMotor;
import org.trigon.hardware.phoenix6.talonfx.TalonFXSignal;
import org.trigon.utilities.Conversions;

public class Pitcher extends MotorSubsystem {
    private final ShootingCalculations shootingCalculations = ShootingCalculations.getInstance();
    private final TalonFXMotor motor = PitcherConstants.MOTOR;
    private final MotionMagicVoltage motionMagicPositionRequest = new MotionMagicVoltage(0).withEnableFOC(PitcherConstants.FOC_ENABLED).withUpdateFreqHz(1000);
    private final VoltageOut voltageRequest = new VoltageOut(0);
    private Rotation2d targetPitch = null;

    public Pitcher() {
        setName("Pitcher");
    }

    @Override
    public void stop() {
        motor.stopMotor();
    }

    @Override
    public void updatePeriodically() {
        motor.update();
        PitcherConstants.ENCODER.update();
        var a = Conversions.rotationsToDegrees(motor.getSignal(TalonFXSignal.CLOSED_LOOP_REFERENCE) - PitcherConstants.GRAVITY_POSITION_TO_REAL_POSITION);
        Logger.recordOutput("CurrentPitchDegrees", Conversions.rotationsToDegrees(motor.getSignal(TalonFXSignal.POSITION) - PitcherConstants.GRAVITY_POSITION_TO_REAL_POSITION));
        if (!DriverStation.isDisabled() && Math.abs(a - 478.8756) > 0.01)
            Logger.recordOutput("TargetPitchDegrees", a);
    }

    @Override
    public void updateMechanism() {
        PitcherConstants.MECHANISM.update(getCurrentPitch(), Rotation2d.fromRotations(motor.getSignal(TalonFXSignal.CLOSED_LOOP_REFERENCE)));
        Logger.recordOutput("Poses/Components/PitcherPose", getPitcherComponentPose());
    }

    @Override
    public void drive(Measure<Voltage> voltageMeasure) {
        motor.setControl(voltageRequest.withOutput(voltageMeasure.in(Units.Volts)));
    }

    @Override
    public void updateLog(SysIdRoutineLog log) {
        log.motor("Pitcher")
                .voltage(Units.Volts.of(motor.getSignal(TalonFXSignal.MOTOR_VOLTAGE)))
                .angularPosition(Units.Rotations.of(PitcherConstants.ENCODER.getSignal(CANcoderSignal.POSITION)))
                .angularVelocity(Units.RotationsPerSecond.of(motor.getSignal(TalonFXSignal.ROTOR_VELOCITY) / PitcherConstants.GEAR_RATIO));
    }

    @Override
    public void setBrake(boolean brake) {
        motor.setBrake(brake);
    }

    @Override
    public SysIdRoutine.Config getSysIdConfig() {
        return PitcherConstants.SYS_ID_CONFIG;
    }

    @AutoLogOutput(key = "Pitcher/AtTargetPitch")
    public boolean atTargetPitch() {
        return targetPitch != null && Math.abs(getCurrentPitch().getDegrees() - targetPitch.getDegrees()) < PitcherConstants.PITCH_TOLERANCE_DEGREES;
    }

    public Rotation2d getTargetPitch() {
        return targetPitch;
    }

    public Rotation2d getCurrentPitch() {
        return Rotation2d.fromRotations(motor.getSignal(TalonFXSignal.POSITION) - PitcherConstants.GRAVITY_POSITION_TO_REAL_POSITION);
    }

    public double getRotorPosition() {
        return motor.getSignal(TalonFXSignal.ROTOR_POSITION);
    }

    public double getEncoderPosition() {
        return PitcherConstants.ENCODER.getSignal(CANcoderSignal.POSITION) - PitcherConstants.GRAVITY_POSITION_TO_REAL_POSITION;
    }

    void pitchToShootingTarget() {
        setTargetPitch(shootingCalculations.getTargetShootingState().targetPitch());
    }

    void setTargetPitch(Rotation2d targetPitch) {
        if (targetPitch == null || Double.isNaN(targetPitch.getRadians())) {
            this.targetPitch = null;
            return;
        }

        motor.setControl(motionMagicPositionRequest.withPosition(targetPitch.getRotations() + PitcherConstants.GRAVITY_POSITION_TO_REAL_POSITION));
        Logger.recordOutput("Pitcher/TargetPitch", targetPitch.getDegrees());
        this.targetPitch = targetPitch;
    }

    private Pose3d getPitcherComponentPose() {
        final Transform3d pitcherTransform = new Transform3d(
                new Translation3d(),
                new Rotation3d(0, getCurrentPitch().getRadians(), 0)
        );
        return PitcherConstants.PITCHER_ORIGIN_POINT.transformBy(pitcherTransform);
    }
}

