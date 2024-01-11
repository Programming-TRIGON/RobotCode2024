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
            currentVelocityLigament,
            currentVelocityTopArrowLigament,
            currentVelocityBottomArrowLigament,
            targetVelocityLigament,
            targetVelocityTopArrowLigament,
            targetVelocityBottomArrowLigament;

    public SpeedMechanism2d(String key, double maxDisplayableVelocity) {
        this.key = key;
        this.mechanism = new Mechanism2d(2 * maxDisplayableVelocity, 2 * maxDisplayableVelocity);
        MechanismRoot2d root = mechanism.getRoot(key, maxDisplayableVelocity, maxDisplayableVelocity);
        this.currentVelocityLigament = root.append(new MechanismLigament2d("ZCurrentVelocityLigament", 0, 0, 5, new Color8Bit(Color.kBlue)));
        this.currentVelocityTopArrowLigament = currentVelocityLigament.append(new MechanismLigament2d("ZCurrentVelocityTopArrowLigament", 0.2 * maxDisplayableVelocity, 210, 5, new Color8Bit(Color.kBlue)));
        this.currentVelocityBottomArrowLigament = currentVelocityLigament.append(new MechanismLigament2d("ZCurrentVelocityBottomArrowLigament", 0.2 * maxDisplayableVelocity, 145, 5, new Color8Bit(Color.kBlue)));

        this.targetVelocityLigament = root.append(new MechanismLigament2d("TargetVelocityLigament", 0, 0, 5, new Color8Bit(Color.kGray)));
        this.targetVelocityTopArrowLigament = targetVelocityLigament.append(new MechanismLigament2d("TargetVelocityTopArrowLigament", 0.2 * maxDisplayableVelocity, 210, 5, new Color8Bit(Color.kGray)));
        this.targetVelocityBottomArrowLigament = targetVelocityLigament.append(new MechanismLigament2d("TargetVelocityBottomArrowLigament", 0.2 * maxDisplayableVelocity, 145, 5, new Color8Bit(Color.kGray)));
    }

    public void setVelocity(double velocity, double targetVelocity) {
        setTargetVelocity(targetVelocity);
        setVelocity(velocity);
    }

    public void setVelocity(double velocity) {
        setCurrentVelocityArrowAngle(velocity);
        currentVelocityLigament.setLength(velocity);
        currentVelocityLigament.setColor(velocityToColor(velocity));
        currentVelocityTopArrowLigament.setColor(velocityToColor(velocity));
        currentVelocityBottomArrowLigament.setColor(velocityToColor(velocity));
        Logger.recordOutput(key, mechanism);
    }

    public void setTargetVelocity(double targetVelocity) {
        setTargetVelocityArrowAngle(targetVelocity);
        targetVelocityLigament.setLength(targetVelocity);
    }

    private Color8Bit velocityToColor(double velocity) {
        if (velocity > 0)
            return new Color8Bit(Color.kGreen);
        else if (velocity < 0)
            return new Color8Bit(Color.kRed);
        return new Color8Bit(Color.kBlue);
    }

    private void setCurrentVelocityArrowAngle(double velocity) {
        if (velocity < 0) {
            currentVelocityTopArrowLigament.setAngle(45);
            currentVelocityBottomArrowLigament.setAngle(315);
        } else if (velocity == 0) {
            currentVelocityTopArrowLigament.setAngle(90);
            currentVelocityBottomArrowLigament.setAngle(270);
        }
    }

    private void setTargetVelocityArrowAngle(double targetVelocity) {
        if (targetVelocity < 0) {
            targetVelocityTopArrowLigament.setAngle(45);
            targetVelocityBottomArrowLigament.setAngle(315);
        } else if (targetVelocity == 0) {
            targetVelocityTopArrowLigament.setAngle(90);
            targetVelocityBottomArrowLigament.setAngle(270);
        }
    }
}