package frc.trigon.robot.subsystems.roller;

public class RollerConstants {
    public enum RollerState {
        DEFAULT(0),
        COLLECTION(0),
        FEEDING(0),
        SCORE_AMP(0),
        SCORE_TRAP(0);

        final double velocityRevolutionsPerSecond;

        RollerState(double velocityRevolutionsPerSecond) {
            this.velocityRevolutionsPerSecond = velocityRevolutionsPerSecond;
        }
    }
}
