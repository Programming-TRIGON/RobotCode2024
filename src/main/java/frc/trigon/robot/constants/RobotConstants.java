package frc.trigon.robot.constants;

import frc.trigon.robot.Robot;
import frc.trigon.robot.utilities.FilesHandler;

public class RobotConstants {
    private static final RobotType UNFILTERED_ROBOT_TYPE = RobotType.TRIUMPH;
    @SuppressWarnings("All")
    public static final RobotType ROBOT_TYPE = Robot.IS_REAL && UNFILTERED_ROBOT_TYPE == RobotType.SIMULATION ? RobotType.TRIUMPH : UNFILTERED_ROBOT_TYPE;
    private static final boolean UNFILTERED_IS_REPLAY = false;
    public static final boolean IS_REPLAY = !Robot.IS_REAL && UNFILTERED_IS_REPLAY;
    public static final double PERIODIC_TIME_SECONDS = 0.02;
    public static final String CANIVORE_NAME = "CANivore";

    public enum RobotType {
        TRIUMPH("/media/sda1/akitlogs/"),
        TRIHARD(FilesHandler.DEPLOY_PATH + "logs/"),
        SIMULATION(FilesHandler.DEPLOY_PATH + "logs/");

        public final String loggingPath;

        RobotType(String loggingPath) {
            this.loggingPath = loggingPath;
        }
    }
}
