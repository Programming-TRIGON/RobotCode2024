package frc.trigon.robot.constants;

import frc.trigon.robot.utilities.FilesHandler;

public class RobotConstants {
    public static final RobotType ROBOT_TYPE = RobotType.SIMULATION;
    public static final boolean IS_REPLAY = false;
    public static final double PERIODIC_TIME_SECONDS = 0.02;
    public static final String CANIVORE_NAME = "canivore";

    public enum RobotType {
        PLACEHOLDER("/media/sda1/logs/"),
        TRIHARD(FilesHandler.DEPLOY_PATH + "logs/"),
        SIMULATION(FilesHandler.DEPLOY_PATH + "logs/");

        public final String loggingPath;

        RobotType(String loggingPath) {
            this.loggingPath = loggingPath;
        }
    }
}
