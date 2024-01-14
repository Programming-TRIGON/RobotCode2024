package frc.trigon.robot.subsystems.pitcher;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.units.Measure;
import edu.wpi.first.units.Units;
import edu.wpi.first.units.Voltage;
import edu.wpi.first.wpilibj.sysid.SysIdRoutineLog;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.trigon.robot.subsystems.MotorSubsystem;
import org.littletonrobotics.junction.Logger;

public class Pitcher extends MotorSubsystem {
    private static final Pitcher INSTANCE = new Pitcher();
    private final PitcherInputsAutoLogged pitcherInputs = new PitcherInputsAutoLogged();
    private final PitcherIO pitcherIO = PitcherIO.generateIO();
    private Rotation2d targetPitch = new Rotation2d();

    public static Pitcher getInstance() {
        return INSTANCE;
    }

    private Pitcher() {
        setName("Pitcher");
    }

    @Override
    public void stop() {
        pitcherIO.stop();
    }

    @Override
    public void periodic() {
        pitcherIO.updateInputs(pitcherInputs);
        Logger.processInputs("Pitcher", pitcherInputs);
        updateMechanism();
    }

    @Override
    public void setBrake(boolean brake) {
        pitcherIO.setBrake(brake);
    }

    @Override
    public void drive(Measure<Voltage> voltageMeasure) {
        pitcherIO.setTargetVoltage(voltageMeasure.in(Units.Volts));
    }

    @Override
    public void updateLog(SysIdRoutineLog log) {
        log.motor("Pitcher")
                .voltage(Units.Volts.of(pitcherInputs.voltage))
                .angularPosition(Units.Degrees.of(pitcherInputs.pitchDegrees))
                .angularVelocity(Units.DegreesPerSecond.of(pitcherInputs.velocityDegreesPerSecond));
    }

    @Override
    public SysIdRoutine.Config getSysIdConfig() {
        return PitcherConstants.SYS_ID_CONFIG;
    }

    public boolean atTargetPitch() {
        return Math.abs(pitcherInputs.pitchDegrees - targetPitch.getDegrees()) < PitcherConstants.PITCH_TOLERANCE_DEGREES;
    }

    void pitchToSpeaker(double distanceToSpeaker) {
        setTargetPitch(calculatePitchToSpeaker(distanceToSpeaker));
    }

    void setTargetPitch(Rotation2d targetPitch) {
        pitcherIO.setTargetPitch(targetPitch);
        this.targetPitch = targetPitch;
    }

    private Rotation2d calculatePitchToSpeaker(double distanceToSpeaker) {
        return Rotation2d.fromRotations(PitcherConstants.PITCH_INTERPOLATION.predict(distanceToSpeaker));
    }

    private void updateMechanism() {
        PitcherConstants.PITCHER_LIGAMENT.setAngle(pitcherInputs.pitchDegrees);
        PitcherConstants.TARGET_PITCHER_LIGAMENT.setAngle(pitcherInputs.profiledSetpointDegrees);

        Logger.recordOutput("Mechanisms/PitcherMechanism", PitcherConstants.PITCHER_MECHANISM);
    }
}

