package frc.trigon.robot.subsystems.shooter;

import com.ctre.phoenix6.controls.TorqueCurrentFOC;
import com.ctre.phoenix6.controls.VelocityTorqueCurrentFOC;
import edu.wpi.first.units.Measure;
import edu.wpi.first.units.Units;
import edu.wpi.first.units.Voltage;
import edu.wpi.first.wpilibj.sysid.SysIdRoutineLog;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.trigon.robot.hardware.phoenix6.talonfx.TalonFXMotor;
import frc.trigon.robot.hardware.phoenix6.talonfx.TalonFXSignal;
import frc.trigon.robot.subsystems.MotorSubsystem;
import frc.trigon.robot.utilities.ShootingCalculations;
import org.littletonrobotics.junction.AutoLogOutput;

public class Shooter extends MotorSubsystem {
    private final ShootingCalculations shootingCalculations = ShootingCalculations.getInstance();
    private final TalonFXMotor motor = ShooterConstants.MASTER_MOTOR;
    private final VelocityTorqueCurrentFOC velocityRequest = new VelocityTorqueCurrentFOC(0);
    private final TorqueCurrentFOC torqueCurrentRequest = new TorqueCurrentFOC(0);
    private double targetVelocityRevolutionsPerSecond = 0;

    public Shooter() {
        setName("Shooter");
    }

    @Override
    public void stop() {
        motor.stopMotor();
        targetVelocityRevolutionsPerSecond = 0;
    }

    @Override
    public void drive(Measure<Voltage> voltageMeasure) {
        motor.setControl(torqueCurrentRequest.withOutput(voltageMeasure.in(Units.Volts)));
    }

    @Override
    public void updateLog(SysIdRoutineLog log) {
        log.motor("Shooter")
                .angularPosition(Units.Rotations.of(motor.getSignal(TalonFXSignal.POSITION)))
                .angularVelocity(Units.RotationsPerSecond.of(motor.getSignal(TalonFXSignal.VELOCITY)))
                .voltage(Units.Volts.of(motor.getSignal(TalonFXSignal.TORQUE_CURRENT)));
    }

    @Override
    public SysIdRoutine.Config getSysIdConfig() {
        return ShooterConstants.SYS_ID_CONFIG;
    }

    @Override
    public void periodic() {
        motor.update();
        updateMechanism();
    }

    @AutoLogOutput(key = "Shooter/AtShootingVelocity")
    public boolean atTargetShootingVelocity() {
        return Math.abs(getCurrentVelocityRevolutionsPerSecond() - targetVelocityRevolutionsPerSecond) < ShooterConstants.TOLERANCE_REVOLUTIONS_PER_SECOND;
    }

    public double getTargetVelocityRevolutionsPerSecond() {
        return targetVelocityRevolutionsPerSecond;
    }

    public double getCurrentVelocityRevolutionsPerSecond() {
        return motor.getSignal(TalonFXSignal.VELOCITY);
    }

    void reachTargetShootingVelocity() {
        final double targetVelocityRevolutionsPerSecond = shootingCalculations.getTargetShootingState().targetShootingVelocityRevolutionsPerSecond();
        setTargetVelocity(targetVelocityRevolutionsPerSecond);
    }

    void setTargetVelocity(double targetVelocityRevolutionsPerSecond) {
        motor.setControl(velocityRequest.withVelocity(targetVelocityRevolutionsPerSecond));
        this.targetVelocityRevolutionsPerSecond = targetVelocityRevolutionsPerSecond;
    }

    private void updateMechanism() {
        ShooterConstants.SHOOTING_MECHANISM.update(getCurrentVelocityRevolutionsPerSecond(), targetVelocityRevolutionsPerSecond);
    }
}

