package frc.trigon.robot.constants;

import frc.trigon.robot.Robot;
import frc.trigon.robot.utilities.FilesHandler;

public class RobotConstants {
    public static final RobotType ROBOT_TYPE = RobotType.TRIUMPH;
    private static final boolean REPLAY = true;
    public static final boolean IS_REPLAY = !Robot.IS_REAL && REPLAY;
    public static final double PERIODIC_TIME_SECONDS = 0.02;
    public static final String CANIVORE_NAME = "CANivore";

    public enum RobotType {
        TRIUMPH("/media/sda1/logs/"),
        TRIHARD(FilesHandler.DEPLOY_PATH + "logs/"),
        SIMULATION(FilesHandler.DEPLOY_PATH + "logs/");

        public final String loggingPath;

        RobotType(String loggingPath) {
            this.loggingPath = loggingPath;
        }
    }
}
