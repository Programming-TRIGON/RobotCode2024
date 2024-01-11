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
    private final MechanismLigament2d
            mainLigament,
            mainTopArrowLigament,
            mainBottomArrowLigament,
            targetVelocityLigament,
            targetVelocityTopArrowLigament,
            targetVelocityBottomArrowLigament;

    public SpeedMechanism2d(String key, double maxDisplayableVelocity) {
        this.key = key;
        this.mechanism = new Mechanism2d(2 * maxDisplayableVelocity, 2 * maxDisplayableVelocity);
        MechanismRoot2d root = mechanism.getRoot(key, maxDisplayableVelocity, maxDisplayableVelocity);
        this.mainLigament = root.append(new MechanismLigament2d("Z" + key, 0, 0, 5, new Color8Bit(Color.kBlue)));
        this.mainTopArrowLigament = mainLigament.append(new MechanismLigament2d(key + "TopArrow", 0.2 * maxDisplayableVelocity, 210, 5, new Color8Bit(Color.kBlue)));
        this.mainBottomArrowLigament = mainLigament.append(new MechanismLigament2d(key + "BottomArrow", 0.2 * maxDisplayableVelocity, 145, 5, new Color8Bit(Color.kBlue)));

        this.targetVelocityLigament = root.append(new MechanismLigament2d(key + "TargetVelocity", 0, 0, 5, new Color8Bit(Color.kGray)));
        this.targetVelocityTopArrowLigament = targetVelocityLigament.append(new MechanismLigament2d(key + "TargetVelocityTopArrow", 0.2 * maxDisplayableVelocity, 210, 5, new Color8Bit(Color.kGray)));
        this.targetVelocityBottomArrowLigament = targetVelocityLigament.append(new MechanismLigament2d(key + "TargetVelocityBottomArrow", 0.2 * maxDisplayableVelocity, 145, 5, new Color8Bit(Color.kGray)));
    }

    public void setVelocity(double velocity, double targetVelocity) {
        mainLigament.setLength(velocity);
        mainLigament.setColor(velocityToColor(velocity));
        mainTopArrowLigament.setColor(velocityToColor(velocity));
        mainBottomArrowLigament.setColor(velocityToColor(velocity));
        targetVelocityLigament.setLength(targetVelocity);
        Logger.recordOutput(key, mechanism);
    }

    public void setVelocity(double velocity) {
        mainLigament.setLength(velocity);
        mainLigament.setColor(velocityToColor(velocity));
        mainTopArrowLigament.setColor(velocityToColor(velocity));
        mainBottomArrowLigament.setColor(velocityToColor(velocity));
        Logger.recordOutput(key, mechanism);
    }

    public void setTargetVelocity(double targetVelocity) {
        targetVelocityLigament.setLength(targetVelocity);
        Logger.recordOutput(key, mechanism);
    }

    private Color8Bit velocityToColor(double velocity) {
        if (velocity > 0)
            return new Color8Bit(Color.kGreen);
        else if (velocity < 0) {
            mainTopArrowLigament.setAngle(45);
            mainBottomArrowLigament.setAngle(315);
            targetVelocityTopArrowLigament.setAngle(45);
            targetVelocityBottomArrowLigament.setAngle(315);
            return new Color8Bit(Color.kRed);
        }
        mainTopArrowLigament.setAngle(90);
        mainBottomArrowLigament.setAngle(270);
        targetVelocityTopArrowLigament.setAngle(90);
        targetVelocityBottomArrowLigament.setAngle(270);
        return new Color8Bit(Color.kBlue);
    }
}