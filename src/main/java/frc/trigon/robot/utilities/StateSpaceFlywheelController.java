package frc.trigon.robot.utilities;

import edu.wpi.first.math.Nat;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.controller.LinearQuadraticRegulator;
import edu.wpi.first.math.estimator.KalmanFilter;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.system.LinearSystem;
import edu.wpi.first.math.system.LinearSystemLoop;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.math.util.Units;
import frc.trigon.robot.constants.RobotConstants;

public class StateSpaceFlywheelController {
    private final LinearSystemLoop<N1, N1, N1> loop;
    private double setpointRadiansPerSecond = 0;

    public StateSpaceFlywheelController(DCMotor gearbox, double momentOfInertia, double gearRatio, double modelAccuracy, double encoderAccuracy, double maximumErrorTolerance, double maximumControlEffort) {
        loop = createLoop(gearbox, momentOfInertia, gearRatio, modelAccuracy, encoderAccuracy, maximumErrorTolerance, maximumControlEffort);
    }

    public StateSpaceFlywheelController(double kv, double ka, double modelAccuracy, double encoderAccuracy, double maximumErrorTolerance, double maximumControlEffort) {
        loop = createLoop(kv, ka, modelAccuracy, encoderAccuracy, maximumErrorTolerance, maximumControlEffort);
    }

    public void reset(double measuredVelocityRevolutionsPerSecond) {
        final double measuredVelocityRadiansPerSecond = Units.rotationsToRadians(measuredVelocityRevolutionsPerSecond);
        loop.reset(VecBuilder.fill(measuredVelocityRadiansPerSecond));
    }

    public double calculate(double measuredVelocityRevolutionsPerSecond, double setpointRevolutionsPerSecond) {
        setSetpoint(setpointRevolutionsPerSecond);
        return calculate(measuredVelocityRevolutionsPerSecond);
    }

    public double calculate(double measuredVelocityRevolutionsPerSecond) {
        final double measuredVelocityRadiansPerSecond = Units.rotationsToRadians(measuredVelocityRevolutionsPerSecond);
        loop.setNextR(VecBuilder.fill(setpointRadiansPerSecond));
        loop.correct(VecBuilder.fill(measuredVelocityRadiansPerSecond));
        loop.predict(RobotConstants.PERIODIC_TIME_SECONDS);
        return loop.getU(0);
    }

    public void setSetpoint(double setpointRevolutionsPerSecond) {
        setpointRadiansPerSecond = Units.rotationsToRadians(setpointRevolutionsPerSecond);
    }

    private LinearSystemLoop<N1, N1, N1> createLoop(DCMotor gearbox, double momentOfInertia, double gearRatio, double modelAccuracy, double encoderAccuracy, double maximumErrorTolerance, double maximumControlEffort) {
        final LinearSystem<N1, N1, N1> flywheelPlant = LinearSystemId.createFlywheelSystem(gearbox, momentOfInertia, gearRatio);
        final LinearQuadraticRegulator<N1, N1, N1> controller = new LinearQuadraticRegulator<>(flywheelPlant, VecBuilder.fill(maximumErrorTolerance), VecBuilder.fill(maximumControlEffort), RobotConstants.PERIODIC_TIME_SECONDS);
        final KalmanFilter<N1, N1, N1> observer = new KalmanFilter<>(Nat.N1(), Nat.N1(), flywheelPlant, VecBuilder.fill(modelAccuracy), VecBuilder.fill(encoderAccuracy), RobotConstants.PERIODIC_TIME_SECONDS);

        return new LinearSystemLoop<>(flywheelPlant, controller, observer, 12.0, RobotConstants.PERIODIC_TIME_SECONDS);
    }

    private LinearSystemLoop<N1, N1, N1> createLoop(double kv, double ka, double modelAccuracy, double encoderAccuracy, double maximumErrorTolerance, double maximumControlEffort) {
        final LinearSystem<N1, N1, N1> flywheelPlant = LinearSystemId.identifyVelocitySystem(kv, ka);
        final LinearQuadraticRegulator<N1, N1, N1> controller = new LinearQuadraticRegulator<>(flywheelPlant, VecBuilder.fill(maximumErrorTolerance), VecBuilder.fill(maximumControlEffort), RobotConstants.PERIODIC_TIME_SECONDS);
        final KalmanFilter<N1, N1, N1> observer = new KalmanFilter<>(Nat.N1(), Nat.N1(), flywheelPlant, VecBuilder.fill(modelAccuracy), VecBuilder.fill(encoderAccuracy), RobotConstants.PERIODIC_TIME_SECONDS);

        return new LinearSystemLoop<>(flywheelPlant, controller, observer, 12.0, RobotConstants.PERIODIC_TIME_SECONDS);
    }
}
