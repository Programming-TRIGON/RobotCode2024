package frc.trigon.robot.subsystems.roller;


import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Roller extends SubsystemBase {
    private final static Roller INSTANCE = new Roller();

    public static Roller getInstance() {
        return INSTANCE;
    }

    private Roller() {
    }
}

