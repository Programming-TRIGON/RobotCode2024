package frc.trigon.robot.subsystems.swerve.simulationswerve;

import edu.wpi.first.math.system.plant.DCMotor;
import frc.trigon.robot.simulation.MotorSimulationConfiguration;
import frc.trigon.robot.simulation.SimpleMotorSimulation;
import frc.trigon.robot.utilities.Conversions;

public class SimulationSwerveModuleConstants {
    static final double WHEEL_DIAMETER_METERS = 0.1016;
    static final double MAX_SPEED_REVOLUTIONS_PER_SECOND = Conversions.distanceToRevolutions(SimulationSwerveConstants.MAX_SPEED_METERS_PER_SECOND, WHEEL_DIAMETER_METERS);
    static final double VOLTAGE_COMPENSATION_SATURATION = 12;

    static final double
            DRIVE_GEAR_RATIO = 8.14,
            STEER_GEAR_RATIO = 12.8;

    private static final double
            DRIVE_MOMENT_OF_INERTIA = 0.003,
            STEER_MOMENT_OF_INERTIA = 0.003;
    static final double
            STEER_MOTOR_P = 72,
            STEER_MOTOR_I = 0,
            STEER_MOTOR_D = 0;
    private static final DCMotor
            DRIVE_MOTOR_GEARBOX = DCMotor.getFalcon500Foc(1),
            STEER_MOTOR_GEARBOX = DCMotor.getFalcon500Foc(1);
    private static final SimpleMotorSimulation
            FRONT_LEFT_DRIVE_MOTOR = new SimpleMotorSimulation(DRIVE_MOTOR_GEARBOX, DRIVE_GEAR_RATIO, DRIVE_MOMENT_OF_INERTIA),
            FRONT_RIGHT_DRIVE_MOTOR = new SimpleMotorSimulation(DRIVE_MOTOR_GEARBOX, DRIVE_GEAR_RATIO, DRIVE_MOMENT_OF_INERTIA),
            REAR_LEFT_DRIVE_MOTOR = new SimpleMotorSimulation(DRIVE_MOTOR_GEARBOX, DRIVE_GEAR_RATIO, DRIVE_MOMENT_OF_INERTIA),
            REAR_RIGHT_DRIVE_MOTOR = new SimpleMotorSimulation(DRIVE_MOTOR_GEARBOX, DRIVE_GEAR_RATIO, DRIVE_MOMENT_OF_INERTIA);
    private static final SimpleMotorSimulation
            FRONT_LEFT_STEER_MOTOR = new SimpleMotorSimulation(STEER_MOTOR_GEARBOX, STEER_GEAR_RATIO, STEER_MOMENT_OF_INERTIA),
            FRONT_RIGHT_STEER_MOTOR = new SimpleMotorSimulation(STEER_MOTOR_GEARBOX, STEER_GEAR_RATIO, STEER_MOMENT_OF_INERTIA),
            REAR_LEFT_STEER_MOTOR = new SimpleMotorSimulation(STEER_MOTOR_GEARBOX, STEER_GEAR_RATIO, STEER_MOMENT_OF_INERTIA),
            REAR_RIGHT_STEER_MOTOR = new SimpleMotorSimulation(STEER_MOTOR_GEARBOX, STEER_GEAR_RATIO, STEER_MOMENT_OF_INERTIA);

    static final SimulationSwerveModuleConstants
            FRONT_LEFT_SWERVE_MODULE_CONSTANTS = new SimulationSwerveModuleConstants(
            FRONT_LEFT_DRIVE_MOTOR,
            FRONT_LEFT_STEER_MOTOR
    ),
            FRONT_RIGHT_SWERVE_MODULE_CONSTANTS = new SimulationSwerveModuleConstants(
                    FRONT_RIGHT_DRIVE_MOTOR,
                    FRONT_RIGHT_STEER_MOTOR
            ),
            REAR_LEFT_SWERVE_MODULE_CONSTANTS = new SimulationSwerveModuleConstants(
                    REAR_LEFT_DRIVE_MOTOR,
                    REAR_LEFT_STEER_MOTOR
            ),
            REAR_RIGHT_SWERVE_MODULE_CONSTANTS = new SimulationSwerveModuleConstants(
                    REAR_RIGHT_DRIVE_MOTOR,
                    REAR_RIGHT_STEER_MOTOR
            );

    final SimpleMotorSimulation driveMotor, steerMotor;

    private SimulationSwerveModuleConstants(SimpleMotorSimulation driveMotor, SimpleMotorSimulation steerMotor) {
        this.driveMotor = driveMotor;
        this.steerMotor = steerMotor;

        configureDriveMotor();
        configureSteerMotor();
    }

    private void configureDriveMotor() {
        final MotorSimulationConfiguration config = new MotorSimulationConfiguration();

        config.conversionFactor = 1;
        config.voltageCompensationSaturation = VOLTAGE_COMPENSATION_SATURATION;

        driveMotor.applyConfiguration(config);
    }

    private void configureSteerMotor() {
        final MotorSimulationConfiguration config = new MotorSimulationConfiguration();

        config.pidConfigs.kP = STEER_MOTOR_P;
        config.pidConfigs.kI = STEER_MOTOR_I;
        config.pidConfigs.kD = STEER_MOTOR_D;
        config.pidConfigs.enableContinuousInput = true;
        config.voltageCompensationSaturation = VOLTAGE_COMPENSATION_SATURATION;
        config.conversionFactor = 1;

        steerMotor.applyConfiguration(config);
    }
}
