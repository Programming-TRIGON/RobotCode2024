package frc.trigon.robot.subsystems.roller;

public class RollerConstants {
    public enum RollerState {
        DEFAULT(0),
        INTAKE(1);
        public final double current;

        RollerState(double current) {
            this.current = current;
        }
    }
}
