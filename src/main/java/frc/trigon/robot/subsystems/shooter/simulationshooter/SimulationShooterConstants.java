package frc.trigon.robot.subsystems.shooter.simulationshooter;

import edu.wpi.first.math.system.plant.DCMotor;
import frc.trigon.robot.motorsimulation.MotorSimulationConfiguration;
import frc.trigon.robot.motorsimulation.SimpleMotorSimulation;

public class SimulationShooterConstants {
    private static final double CONVERSIONS_FACTOR = 1;
    private static final double VOLTAGE_COMPENSATION_SATURATION = 12;
    private static final int
            SHOOTING_MOTOR_AMOUNT = 1,
            FEEDING_MOTOR_AMOUNT = 1;
    private static final DCMotor
            SHOOTING_MOTOR_GEARBOX = DCMotor.getKrakenX60Foc(SHOOTING_MOTOR_AMOUNT),
            FEEDING_MOTOR_GEARBOX = DCMotor.getKrakenX60Foc(FEEDING_MOTOR_AMOUNT);
    private static final double
            SHOOTING_GEAR_RATIO = 1,
            FEEDING_GEAR_RATIO = 1;
    private static final double
            SHOOTING_MOMENT_OF_INERTIA = 0.0001,
            FEEDING_MOMENT_OF_INERTIA = 0.0001;
    private static final double
            SHOOTING_MOTOR_KS = 0,
            SHOOTING_MOTOR_KV = 0.12365;
    static final SimpleMotorSimulation
            SHOOTING_MOTOR = new SimpleMotorSimulation(SHOOTING_MOTOR_GEARBOX, SHOOTING_GEAR_RATIO, SHOOTING_MOMENT_OF_INERTIA),
            FEEDING_MOTOR = new SimpleMotorSimulation(FEEDING_MOTOR_GEARBOX, FEEDING_GEAR_RATIO, FEEDING_MOMENT_OF_INERTIA);

    static {
        configureShootingMotor();
        configureFeedingMotor();
    }

    private static void configureShootingMotor() {
        final MotorSimulationConfiguration config = new MotorSimulationConfiguration();

        config.feedforwardConfigs.kS = SHOOTING_MOTOR_KS;
        config.feedforwardConfigs.kV = SHOOTING_MOTOR_KV;
        config.voltageCompensationSaturation = VOLTAGE_COMPENSATION_SATURATION;
        config.conversionsFactor = CONVERSIONS_FACTOR;

        SHOOTING_MOTOR.applyConfiguration(config);
    }

    private static void configureFeedingMotor() {
        final MotorSimulationConfiguration config = new MotorSimulationConfiguration();

        config.voltageCompensationSaturation = VOLTAGE_COMPENSATION_SATURATION;
        config.conversionsFactor = CONVERSIONS_FACTOR;

        FEEDING_MOTOR.applyConfiguration(config);
    }
}
