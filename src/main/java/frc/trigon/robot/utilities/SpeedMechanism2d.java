package frc.trigon.robot.utilities;

import edu.wpi.first.wpilibj.smartdashboard.Mechanism2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismLigament2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismRoot2d;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj.util.Color8Bit;
import org.littletonrobotics.junction.Logger;

public class SpeedMechanism2d {
    private final String key;
    private final Mechanism2d mechanism;
    private final MechanismLigament2d ligament;

    public SpeedMechanism2d(String key, double maxVelocity) {
        this.key = key;
        this.mechanism = new Mechanism2d(2 * maxVelocity, 2 * maxVelocity);
        MechanismRoot2d root = mechanism.getRoot(key, maxVelocity, maxVelocity);
        this.ligament = root.append(new MechanismLigament2d(key, 0, 0, 10, new Color8Bit(Color.kBlue)));
    }

    public void setVelocity(double velocity) {
        ligament.setLength(velocity);
        ligament.setColor(setColor(velocity));
        Logger.recordOutput(key, mechanism);
    }

    private Color8Bit setColor(double velocity) {
        if (velocity > 0)
            return new Color8Bit(Color.kGreen);
        else if (velocity < 0)
            return new Color8Bit(Color.kRed);
        return new Color8Bit(Color.kBlue);
    }
}