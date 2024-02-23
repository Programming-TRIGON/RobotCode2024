package frc.trigon.robot.subsystems.pitcher;

import edu.wpi.first.math.geometry.*;
import edu.wpi.first.units.Measure;
import edu.wpi.first.units.Units;
import edu.wpi.first.units.Voltage;
import edu.wpi.first.wpilibj.sysid.SysIdRoutineLog;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.trigon.robot.subsystems.MotorSubsystem;
import frc.trigon.robot.utilities.ShootingCalculations;
import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;

public class Pitcher extends MotorSubsystem {
    private final ShootingCalculations shootingCalculations = ShootingCalculations.getInstance();
    private final PitcherInputsAutoLogged pitcherInputs = new PitcherInputsAutoLogged();
    private final PitcherIO pitcherIO = PitcherIO.generateIO();
    private Rotation2d targetPitch = new Rotation2d();

    public Pitcher() {
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

    @AutoLogOutput(key = "Pitcher/AtTargetPitch")
    public boolean atTargetPitch() {
        return Math.abs(pitcherInputs.pitchDegrees - targetPitch.getDegrees()) < PitcherConstants.PITCH_TOLERANCE_DEGREES;
    }

    void pitchToSpeaker() {
        setTargetPitch(shootingCalculations.calculateTargetPitch());
    }

    void setTargetPitch(Rotation2d targetPitch) {
        pitcherIO.setTargetPitch(targetPitch);

        this.targetPitch = targetPitch;
    }

    private void updateMechanism() {
        PitcherConstants.PITCHER_LIGAMENT.setAngle(pitcherInputs.pitchDegrees);
        PitcherConstants.TARGET_PITCHER_LIGAMENT.setAngle(pitcherInputs.profiledSetpointDegrees);

        Logger.recordOutput("Mechanisms/PitcherMechanism", PitcherConstants.PITCHER_MECHANISM);
        Logger.recordOutput("Poses/Components/PitcherPose", getPitcherComponentPose());
    }

    private Pose3d getPitcherComponentPose() {
        final Transform3d pitcherTransform = new Transform3d(
                new Translation3d(),
                new Rotation3d(0, edu.wpi.first.math.util.Units.degreesToRadians(pitcherInputs.pitchDegrees), 0)
        );
        return PitcherConstants.PITCHER_ORIGIN_POINT.transformBy(pitcherTransform);
    }
}

