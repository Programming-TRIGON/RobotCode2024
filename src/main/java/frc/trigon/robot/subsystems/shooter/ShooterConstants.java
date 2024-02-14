package frc.trigon.robot.subsystems.shooter;

import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.units.Units;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.trigon.robot.utilities.SpeedMechanism2d;
import frc.trigon.robot.utilities.StateSpaceFlywheelController;

public class ShooterConstants {
    static final double TOLERANCE_REVOLUTIONS = 0.1;

    static final double SHOOTING_CURRENT = 40;
    static final double SHOOTING_TIME_THRESHOLD = 0.5;
    static final double
            SHOOTING_RUMBLE_DURATION_SECONDS = 0.4,
            SHOOTING_RUMBLE_POWER = 1;

    static final SysIdRoutine.Config SYS_ID_CONFIG = new SysIdRoutine.Config(
            Units.Volts.of(0.25).per(Units.Second),
            Units.Volts.of(7),
            Units.Second.of(1000000000)
    );

    private static final double MAX_DISPLAYABLE_VELOCITY = 120;
    static final SpeedMechanism2d SHOOTING_MECHANISM = new SpeedMechanism2d("Mechanisms/ShooterMechanism", MAX_DISPLAYABLE_VELOCITY);

    public static final double GEAR_RATIO = 1;
    public static final double
            KV = 0.14142,
            KA = 0.038638;
    private static final int MOTOR_AMOUNT = 2;
    public static final DCMotor GEARBOX = DCMotor.getKrakenX60Foc(MOTOR_AMOUNT);
    private static final double
            MODEL_ACCURACY = 3.0,
            ENCODER_ACCURACY = 0.01;
    private static final double
            MAXIMUM_ERROR_TOLERANCE = 8,
            MAXIMUM_CONTROL_EFFORT = 12;
    static final SimpleMotorFeedforward FEEDFORWARD = new SimpleMotorFeedforward(
            0.59843,
            KV,
            KA
    );
    static final StateSpaceFlywheelController STATE_SPACE_CONTROLLER = new StateSpaceFlywheelController(
            KV,
            KA,
            MODEL_ACCURACY,
            ENCODER_ACCURACY,
            MAXIMUM_ERROR_TOLERANCE,
            MAXIMUM_CONTROL_EFFORT
    );
}
