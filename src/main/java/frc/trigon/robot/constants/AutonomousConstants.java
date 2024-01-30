package frc.trigon.robot.constants;

import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.path.PathConstraints;
import frc.trigon.robot.subsystems.collector.CollectorCommands;
import frc.trigon.robot.subsystems.collector.CollectorConstants;
import frc.trigon.robot.subsystems.pitcher.PitcherCommands;
import frc.trigon.robot.subsystems.roller.RollerCommands;
import frc.trigon.robot.subsystems.roller.RollerConstants;
import frc.trigon.robot.subsystems.shooter.ShooterCommands;

public class AutonomousConstants {
    public static final PathConstraints REAL_TIME_CONSTRAINTS = new PathConstraints(2.5, 2.5, 4, 4);

    static {
        registerCommands();
    }

    private static void registerCommands() {
        NamedCommands.registerCommand("RollerCollection", RollerCommands.getSetTargetStateCommand(RollerConstants.RollerState.COLLECTING).withTimeout(1));
        NamedCommands.registerCommand("CollectorCollection", CollectorCommands.getSetTargetStateCommand(CollectorConstants.CollectorState.COLLECTING).withTimeout(1));
        NamedCommands.registerCommand("PitcherToSpeaker", PitcherCommands.getPitchToSpeakerCommand().withTimeout(0.7));
        NamedCommands.registerCommand("ShooterShootSpeaker", ShooterCommands.getShootAtSpeakerCommand().withTimeout(0.3));
    }
}