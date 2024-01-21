package frc.trigon.robot.subsystems.roller.simulationroller;

import edu.wpi.first.math.system.plant.DCMotor;
import frc.trigon.robot.simulation.FlywheelSimulation;
import frc.trigon.robot.simulation.MotorSimulationConfiguration;
import frc.trigon.robot.subsystems.roller.RollerConstants;

public class SimulationRollerConstants {
    private static final double VOLTAGE_COMPENSATION_SATURATION = 12;
    private static final double CONVERSIONS_FACTOR = 1;
    private static final int NUMBER_OF_MOTORS = 1;
    private static final double ROLLER_WEIGHT_KG = 0.5;
    private static final DCMotor MOTOR_GEARBOX = DCMotor.getKrakenX60Foc(NUMBER_OF_MOTORS);
    private static final double
            KS = 0,
            KV = 0.123,
            KA = 0;

    static final FlywheelSimulation MOTOR = new FlywheelSimulation(
            MOTOR_GEARBOX,
            RollerConstants.GEAR_RATIO,
            ROLLER_WEIGHT_KG
    );

    static {
        final MotorSimulationConfiguration config = new MotorSimulationConfiguration();

        config.voltageCompensationSaturation = VOLTAGE_COMPENSATION_SATURATION;
        config.conversionsFactor = CONVERSIONS_FACTOR;

        config.feedforwardConfigs.kS = KS;
        config.feedforwardConfigs.kV = KV;
        config.feedforwardConfigs.kA = KA;

        MOTOR.applyConfiguration(config);
    }
}
