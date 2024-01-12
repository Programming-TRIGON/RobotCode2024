package frc.trigon.robot.utilities;

import edu.wpi.first.wpilibj.smartdashboard.Mechanism2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismLigament2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismRoot2d;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj.util.Color8Bit;
import org.littletonrobotics.junction.Logger;

public class SpeedMechanism2d {
    private final double maxDisplayableVelocity;
    private final String key;
    private final Mechanism2d mechanism;
    private final MechanismRoot2d root;
    private MechanismLigament2d
            currentVelocityLigament,
            currentVelocityTopArrowLigament,
            currentVelocityBottomArrowLigament,
            targetVelocityLigament,
            targetVelocityTopArrowLigament,
            targetVelocityBottomArrowLigament;
    private final Color8Bit
            green = new Color8Bit(Color.kGreen),
            red = new Color8Bit(Color.kRed),
            blue = new Color8Bit(Color.kBlue);
    private final double
            negativeTopAngle = 45,
            negativeBottomAngle = 315,
            positiveTopAngle = 210,
            positiveBottomAngle = 145,
            zeroTopAngle = 90,
            zeroBottomAngle = 270;
    private final double deadband = 0.001;
    private double lastTargetVelocity = 0;
    public SpeedMechanism2d(String key, double maxDisplayableVelocity) {
        this.maxDisplayableVelocity = maxDisplayableVelocity;
        this.key = key;
        this.mechanism = new Mechanism2d(2 * maxDisplayableVelocity, 2 * maxDisplayableVelocity);
        this.root = mechanism.getRoot(key, maxDisplayableVelocity, maxDisplayableVelocity);
    }

    public void setVelocity(double velocity) {
        setVelocity(velocity, lastTargetVelocity);
    }

    public void setVelocity(double velocity, double targetVelocity) {
        createLigaments(velocity, targetVelocity);
        setTargetVelocity(targetVelocity);
        setTargetVelocityArrowAngle(targetVelocity);
        targetVelocityLigament.setLength(targetVelocity);
        setCurrentVelocityArrowAngle(velocity);
        currentVelocityLigament.setLength(velocity);
        setLigamentColor(velocity);
        Logger.recordOutput(key, mechanism);
    }

    public void setTargetVelocity(double targetVelocity) {
        lastTargetVelocity = targetVelocity;
    }

    private void createLigaments(double currentVelocity, double targetVelocity) {
        String currentVelocityLigamentKeyZ = "";
        String targetVelocityLigamentKeyZ = "";
        if (isCurrentVelocityLigamentBiggerThanTargetVelocityLigament(currentVelocity, targetVelocity))
            targetVelocityLigamentKeyZ = "Z";
        else
            currentVelocityLigamentKeyZ = "Z";
        this.currentVelocityLigament = root.append(new MechanismLigament2d(currentVelocityLigamentKeyZ + "CurrentVelocityLigament", 0, 0, 5, new Color8Bit(Color.kBlue)));
        this.currentVelocityTopArrowLigament = currentVelocityLigament.append(new MechanismLigament2d(currentVelocityLigamentKeyZ + "CurrentVelocityTopArrowLigament", 0.2 * maxDisplayableVelocity, zeroTopAngle, 5, new Color8Bit(Color.kBlue)));
        this.currentVelocityBottomArrowLigament = currentVelocityLigament.append(new MechanismLigament2d(currentVelocityLigamentKeyZ + "CurrentVelocityBottomArrowLigament", 0.2 * maxDisplayableVelocity, zeroBottomAngle, 5, new Color8Bit(Color.kBlue)));

        this.targetVelocityLigament = root.append(new MechanismLigament2d(targetVelocityLigamentKeyZ + "TargetVelocityLigament", 0, 0, 5, new Color8Bit(Color.kGray)));
        this.targetVelocityTopArrowLigament = targetVelocityLigament.append(new MechanismLigament2d(targetVelocityLigamentKeyZ + "TargetVelocityTopArrowLigament", 0.2 * maxDisplayableVelocity, zeroTopAngle, 5, new Color8Bit(Color.kGray)));
        this.targetVelocityBottomArrowLigament = targetVelocityLigament.append(new MechanismLigament2d(targetVelocityLigamentKeyZ + "TargetVelocityBottomArrowLigament", 0.2 * maxDisplayableVelocity, zeroBottomAngle, 5, new Color8Bit(Color.kGray)));
    }

    private boolean isCurrentVelocityLigamentBiggerThanTargetVelocityLigament(double currentVelocity, double targetVelocity) {
        if (currentVelocity > deadband && targetVelocity > deadband)
            return currentVelocity > targetVelocity;
        else if (currentVelocity < -deadband && targetVelocity < -deadband)
            return currentVelocity < targetVelocity;
        return false;
    }

    private void setLigamentColor(double velocity) {
        currentVelocityLigament.setColor(velocityToColor(velocity));
        currentVelocityTopArrowLigament.setColor(velocityToColor(velocity));
        currentVelocityBottomArrowLigament.setColor(velocityToColor(velocity));
    }

    private Color8Bit velocityToColor(double velocity) {
        if (velocity > deadband)
            return green;
        else if (velocity < -deadband)
            return red;
        return blue;
    }

    private void setCurrentVelocityArrowAngle(double velocity) {
        if (velocity > deadband) {
            currentVelocityTopArrowLigament.setAngle(positiveTopAngle);
            currentVelocityBottomArrowLigament.setAngle(positiveBottomAngle);
        } else if (velocity < -deadband) {
            currentVelocityTopArrowLigament.setAngle(negativeTopAngle);
            currentVelocityBottomArrowLigament.setAngle(negativeBottomAngle);
        }
    }

    private void setTargetVelocityArrowAngle(double targetVelocity) {
        if (targetVelocity > deadband) {
            targetVelocityTopArrowLigament.setAngle(positiveTopAngle);
            targetVelocityBottomArrowLigament.setAngle(positiveBottomAngle);
        } else if (targetVelocity < -deadband) {
            targetVelocityTopArrowLigament.setAngle(negativeTopAngle);
            targetVelocityBottomArrowLigament.setAngle(negativeBottomAngle);
        }
    }
}