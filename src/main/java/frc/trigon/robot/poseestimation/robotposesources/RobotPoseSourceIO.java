package frc.trigon.robot.poseestimation.robotposesources;

import edu.wpi.first.math.geometry.Pose3d;
import org.littletonrobotics.junction.AutoLog;

public class RobotPoseSourceIO {
    protected void updateInputs(RobotPoseSourceInputsAutoLogged inputs) {
    }

    @AutoLog
    public static class RobotPoseSourceInputs {
        public boolean hasResult = false;
        public double lastResultTimestamp = 0;
        public Pose3d cameraPose = new Pose3d();
        public double averageDistanceFromTags = 0;
        public int visibleTags = 0;
    }
}
