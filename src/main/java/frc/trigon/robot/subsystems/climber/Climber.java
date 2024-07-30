package frc.trigon.robot.subsystems.climber;

import com.ctre.phoenix6.controls.DynamicMotionMagicVoltage;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.controls.TorqueCurrentFOC;
import com.ctre.phoenix6.controls.VoltageOut;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.units.Measure;
import edu.wpi.first.units.Units;
import edu.wpi.first.units.Voltage;
import edu.wpi.first.wpilibj.sysid.SysIdRoutineLog;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.trigon.robot.commands.Commands;
import frc.trigon.robot.constants.CommandConstants;
import frc.trigon.robot.hardware.phoenix6.talonfx.TalonFXMotor;
import frc.trigon.robot.hardware.phoenix6.talonfx.TalonFXSignal;
import frc.trigon.robot.subsystems.MotorSubsystem;
import frc.trigon.robot.utilities.Conversions;
import org.littletonrobotics.junction.Logger;

public class Climber extends MotorSubsystem {
    private final TalonFXMotor
            masterMotor = ClimberConstants.MASTER_MOTOR,
            followerMotor = ClimberConstants.FOLLOWER_MOTOR;
    private final TorqueCurrentFOC torqueCurrentRequest = new TorqueCurrentFOC(0);
    private final DynamicMotionMagicVoltage
            nonClimbingPositionRequest = new DynamicMotionMagicVoltage(
            0,
            ClimberConstants.MAX_NON_CLIMBING_VELOCITY,
            ClimberConstants.MAX_NON_CLIMBING_ACCELERATION,
            0).withSlot(ClimberConstants.NON_CLIMBING_SLOT).withEnableFOC(ClimberConstants.ENABLE_FOC
    ),
            climbingPositionRequest = new DynamicMotionMagicVoltage(
                    0,
                    ClimberConstants.MAX_CLIMBING_VELOCITY,
                    ClimberConstants.MAX_CLIMBING_ACCELERATION,
                    0).withSlot(ClimberConstants.CLIMBING_SLOT).withEnableFOC(ClimberConstants.ENABLE_FOC
            );
    private final VoltageOut voltageRequest = new VoltageOut(0).withEnableFOC(ClimberConstants.ENABLE_FOC);
    private final PositionVoltage positionRequest = new PositionVoltage(0).withEnableFOC(ClimberConstants.ENABLE_FOC);
    private ClimberConstants.ClimberState targetState = ClimberConstants.ClimberState.RESTING;

    public Climber() {
        setName("Climber");
        configurePositionResettingLimitSwitch();
        Commands.getDelayedCommand(3, this::configureChangingDefaultCommand).schedule();
    }

    @Override
    public void periodic() {
        masterMotor.update();
        updateNetworkTables();
    }

    @Override
    public void drive(Measure<Voltage> voltageMeasure) {
        masterMotor.setControl(torqueCurrentRequest.withOutput(voltageMeasure.in(Units.Volts)));
    }

    @Override
    public void updateLog(SysIdRoutineLog log) {
        log.motor("Climber")
                .linearPosition(Units.Meters.of(masterMotor.getSignal(TalonFXSignal.POSITION)))
                .linearVelocity(Units.MetersPerSecond.of(masterMotor.getSignal(TalonFXSignal.VELOCITY)))
                .voltage(Units.Volts.of(masterMotor.getSignal(TalonFXSignal.TORQUE_CURRENT)));
    }

    @Override
    public void setBrake(boolean brake) {
        masterMotor.setBrake(brake);
        followerMotor.setBrake(brake);
    }

    @Override
    public SysIdRoutine.Config getSysIdConfig() {
        return ClimberConstants.SYSID_CONFIG;
    }

    @Override
    public void stop() {
        masterMotor.stopMotor();
    }

    public boolean atTargetState() {
        return Math.abs(getPositionMeters() - targetState.positionMeters) < ClimberConstants.TOLERANCE_METERS;
    }

    public ClimberConstants.ClimberState getTargetState() {
        return targetState;
    }

    public boolean isReadyForElevatorOpening() {
        return getPositionMeters() < ClimberConstants.READY_FOR_ELEVATOR_OPENING_MAXIMUM_POSITION_METERS;
    }

    public boolean isLimitSwitchPressed() {
        return ClimberConstants.LIMIT_SWITCH.getBinaryValue();
    }

    void setTargetState(ClimberConstants.ClimberState targetState) {
        setTargetPosition(targetState.positionMeters, targetState.affectedByWeight);
        this.targetState = targetState;
    }

    void setTargetPosition(double targetPositionMeters, boolean affectedByWeight) {
        masterMotor.setControl(determineRequest(affectedByWeight).withPosition(targetPositionMeters));
    }

    private void updateNetworkTables() {
        updateMechanisms();
        Logger.recordOutput("Climber/PositionMeters", getPositionMeters());
        Logger.recordOutput("Climber/VelocityMeters", toMeters(masterMotor.getSignal(TalonFXSignal.VELOCITY)));
    }

    private void updateMechanisms() {
        ClimberConstants.MECHANISM.update(toMeters(
                        masterMotor.getSignal(TalonFXSignal.POSITION)),
                toMeters(masterMotor.getSignal(TalonFXSignal.CLOSED_LOOP_REFERENCE))
        );
        Logger.recordOutput("Poses/Components/ClimberPose", getClimberPose());
    }

    private Pose3d getClimberPose() {
        final Transform3d climberTransform = new Transform3d(
                new Translation3d(0, 0, getPositionMeters()),
                new Rotation3d()
        );
        return ClimberConstants.CLIMBER_ORIGIN_POINT.transformBy(climberTransform);
    }

    private double getPositionMeters() {
        return toMeters(masterMotor.getSignal(TalonFXSignal.POSITION));
    }

    private void configureChangingDefaultCommand() {
        final Trigger climbingTrigger = new Trigger(() -> CommandConstants.IS_CLIMBING);
        climbingTrigger.onTrue(new InstantCommand(this::defaultToClimbing));
        climbingTrigger.onFalse(new InstantCommand(this::defaultToResting));
    }

    private void defaultToResting() {
        changeDefaultCommand(ClimberCommands.getSetTargetStateCommand(ClimberConstants.ClimberState.RESTING));
    }

    private void defaultToClimbing() {
        changeDefaultCommand(ClimberCommands.getStopCommand());
    }

    private double toMeters(double revolutions) {
        return Conversions.revolutionsToDistance(revolutions, ClimberConstants.DRUM_DIAMETER_METERS);
    }

    private double toRevolutions(double meters) {
        return Conversions.distanceToRevolutions(meters, ClimberConstants.DRUM_DIAMETER_METERS);
    }

    private void configurePositionResettingLimitSwitch() {
        final Trigger limitSwitchTrigger = new Trigger(() -> ClimberConstants.LIMIT_SWITCH.getBinaryValue() && !CommandConstants.IS_CLIMBING);
        limitSwitchTrigger.and(() -> masterMotor.getSignal(TalonFXSignal.POSITION) != 0).debounce(ClimberConstants.LIMIT_SWITCH_PRESSED_THRESHOLD_SECONDS).whileTrue(new InstantCommand(this::resetPosition).repeatedly().ignoringDisable(true));
    }

    private DynamicMotionMagicVoltage determineRequest(boolean affectedByWeight) {
        return affectedByWeight ? climbingPositionRequest : nonClimbingPositionRequest;
    }

    private void resetPosition() {
        masterMotor.setControl(positionRequest.withPosition(0));
    }
}
