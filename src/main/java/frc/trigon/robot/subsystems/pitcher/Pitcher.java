package frc.trigon.robot.subsystems.pitcher;

import com.ctre.phoenix6.controls.MotionMagicExpoTorqueCurrentFOC;
import com.ctre.phoenix6.controls.PositionTorqueCurrentFOC;
import com.ctre.phoenix6.controls.TorqueCurrentFOC;
import edu.wpi.first.math.geometry.*;
import edu.wpi.first.units.Measure;
import edu.wpi.first.units.Units;
import edu.wpi.first.units.Voltage;
import edu.wpi.first.wpilibj.sysid.SysIdRoutineLog;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.trigon.robot.hardware.phoenix6.talonfx.TalonFXMotor;
import frc.trigon.robot.hardware.phoenix6.talonfx.TalonFXSignal;
import frc.trigon.robot.subsystems.MotorSubsystem;
import frc.trigon.robot.utilities.ShootingCalculations;
import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;

public class Pitcher extends MotorSubsystem {
    private final ShootingCalculations shootingCalculations = ShootingCalculations.getInstance();
    private final TalonFXMotor motor = PitcherConstants.MOTOR;
    private final MotionMagicExpoTorqueCurrentFOC motionMagicPositionRequest = new MotionMagicExpoTorqueCurrentFOC(0).withSlot(PitcherConstants.MOTION_MAGIC_SLOT).withUpdateFreqHz(1000);
    private final PositionTorqueCurrentFOC positionRequest = new PositionTorqueCurrentFOC(0).withSlot(PitcherConstants.POSITION_SLOT).withUpdateFreqHz(1000);
    private final TorqueCurrentFOC torqueCurrentRequest = new TorqueCurrentFOC(0);
    private Rotation2d targetPitch = null;

    public Pitcher() {
        setName("Pitcher");
//        setupControlTypeSwitching();
    }

    @Override
    public void stop() {
        motor.stopMotor();
        targetPitch = null;
    }

    @Override
    public void periodic() {
        motor.update();
        updateMechanism();
    }

    @Override
    public void drive(Measure<Voltage> voltageMeasure) {
        motor.setControl(torqueCurrentRequest.withOutput(voltageMeasure.in(Units.Volts)));
    }

    @Override
    public void setBrake(boolean brake) {
        motor.setBrake(brake);
    }

    @Override
    public void updateLog(SysIdRoutineLog log) {
        log.motor("Pitcher")
                .voltage(Units.Volts.of(motor.getSignal(TalonFXSignal.TORQUE_CURRENT)))
                .angularPosition(Units.Rotations.of(motor.getSignal(TalonFXSignal.POSITION)))
                .angularVelocity(Units.RotationsPerSecond.of(motor.getSignal(TalonFXSignal.VELOCITY)));
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
        return Rotation2d.fromRotations(motor.getSignal(TalonFXSignal.POSITION));
    }

    void pitchToShootingTarget() {
        setTargetPitch(shootingCalculations.getTargetShootingState().targetPitch());
    }

    void setTargetPitch(Rotation2d targetPitch) {
        if (targetPitch == null)
            return;

        motor.setControl(motionMagicPositionRequest.withPosition(targetPitch.getRotations()));
        Logger.recordOutput("TargetPitch", targetPitch.getDegrees());
        this.targetPitch = targetPitch;
    }

    private boolean shouldUsePositionControl() {
        return targetPitch != null && Math.abs(targetPitch.getDegrees() - getCurrentPitch().getDegrees()) < PitcherConstants.SWITCHING_TO_POSITION_CONTROL_TOLERANCE_DEGREES;
    }

    private void setupControlTypeSwitching() {
        final Trigger controlChangeTrigger = new Trigger(this::shouldUsePositionControl).debounce(0.04);
        controlChangeTrigger.onFalse(new InstantCommand(() -> setTargetPitch(targetPitch)));
        controlChangeTrigger.onTrue(new InstantCommand(() -> setTargetPitch(targetPitch)));
    }

    private void updateMechanism() {
        PitcherConstants.MECHANISM.update(getCurrentPitch(), Rotation2d.fromRotations(motor.getSignal(TalonFXSignal.CLOSED_LOOP_REFERENCE)));
        Logger.recordOutput("Poses/Components/PitcherPose", getPitcherComponentPose());
    }

    private Pose3d getPitcherComponentPose() {
        final Transform3d pitcherTransform = new Transform3d(
                new Translation3d(),
                new Rotation3d(0, getCurrentPitch().getRadians(), 0)
        );
        return PitcherConstants.PITCHER_ORIGIN_POINT.transformBy(pitcherTransform);
    }
}

