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
    private static final Color8Bit
            green = new Color8Bit(Color.kGreen),
            red = new Color8Bit(Color.kRed),
            blue = new Color8Bit(Color.kBlue);
    private double deadband = 0.001;
    private static final double
            negativeTopAngle = 45,
            negativeBottomAngle = 315,
            positiveTopAngle = 210,
            positiveBottomAngle = 145,
            zeroTopAngle = 90,
            zeroBottomAngle = 270;
    private double lastTargetVelocity = 0;

    public SpeedMechanism2d(String key, double maxDisplayableVelocity) {
        this.key = key;
        this.mechanism = new Mechanism2d(2 * maxDisplayableVelocity, 2 * maxDisplayableVelocity);
        MechanismRoot2d root = mechanism.getRoot("VelocityRoot", maxDisplayableVelocity, maxDisplayableVelocity);
        this.currentVelocityLigament = root.append(new MechanismLigament2d("ZCurrentVelocityLigament", 0, 0, 5, new Color8Bit(Color.kBlue)));
        this.currentVelocityTopArrowLigament = currentVelocityLigament.append(new MechanismLigament2d(  "ZCurrentVelocityTopArrowLigament", 0.2 * maxDisplayableVelocity, zeroTopAngle, 5, new Color8Bit(Color.kBlue)));
        this.currentVelocityBottomArrowLigament = currentVelocityLigament.append(new MechanismLigament2d("ZCurrentVelocityBottomArrowLigament", 0.2 * maxDisplayableVelocity, zeroBottomAngle, 5, new Color8Bit(Color.kBlue)));

        this.targetVelocityLigament = root.append(new MechanismLigament2d("TargetVelocityLigament", 0, 0, 5, new Color8Bit(Color.kGray)));
        this.targetVelocityTopArrowLigament = targetVelocityLigament.append(new MechanismLigament2d("TargetVelocityTopArrowLigament", 0.2 * maxDisplayableVelocity, zeroTopAngle, 5, new Color8Bit(Color.kGray)));
        this.targetVelocityBottomArrowLigament = targetVelocityLigament.append(new MechanismLigament2d("TargetVelocityBottomArrowLigament", 0.2 * maxDisplayableVelocity, zeroBottomAngle, 5, new Color8Bit(Color.kGray)));
    }

    public SpeedMechanism2d(String key, double maxDisplayableVelocity, double deadband) {
        this.deadband = deadband;
        this.key = key;
        this.mechanism = new Mechanism2d(2 * maxDisplayableVelocity, 2 * maxDisplayableVelocity);
        MechanismRoot2d root = mechanism.getRoot("VelocityRoot", maxDisplayableVelocity, maxDisplayableVelocity);
        this.currentVelocityLigament = root.append(new MechanismLigament2d("ZCurrentVelocityLigament", 0, 0, 5, new Color8Bit(Color.kBlue)));
        this.currentVelocityTopArrowLigament = currentVelocityLigament.append(new MechanismLigament2d(  "ZCurrentVelocityTopArrowLigament", 0.2 * maxDisplayableVelocity, zeroTopAngle, 5, new Color8Bit(Color.kBlue)));
        this.currentVelocityBottomArrowLigament = currentVelocityLigament.append(new MechanismLigament2d("ZCurrentVelocityBottomArrowLigament", 0.2 * maxDisplayableVelocity, zeroBottomAngle, 5, new Color8Bit(Color.kBlue)));

        this.targetVelocityLigament = root.append(new MechanismLigament2d("TargetVelocityLigament", 0, 0, 5, new Color8Bit(Color.kGray)));
        this.targetVelocityTopArrowLigament = targetVelocityLigament.append(new MechanismLigament2d("TargetVelocityTopArrowLigament", 0.2 * maxDisplayableVelocity, zeroTopAngle, 5, new Color8Bit(Color.kGray)));
        this.targetVelocityBottomArrowLigament = targetVelocityLigament.append(new MechanismLigament2d("TargetVelocityBottomArrowLigament", 0.2 * maxDisplayableVelocity, zeroBottomAngle, 5, new Color8Bit(Color.kGray)));
    }

    public void setVelocity(double velocity) {
        setVelocity(velocity, lastTargetVelocity);
    }

    public void setVelocity(double velocity, double targetVelocity) {
        setTargetVelocity(targetVelocity);
        setArrowAngle(velocity, targetVelocity);
        currentVelocityLigament.setLength(velocity);
        setCurrentLigamentColor(velocityToColor(velocity));
        Logger.recordOutput(key, mechanism);
    }

    public void setTargetVelocity(double targetVelocity) {
        lastTargetVelocity = targetVelocity;
        targetVelocityLigament.setLength(targetVelocity);
    }

    private void setCurrentLigamentColor(Color8Bit color) {
        currentVelocityLigament.setColor(color);
        currentVelocityTopArrowLigament.setColor(color);
        currentVelocityBottomArrowLigament.setColor(color);
    }

    private Color8Bit velocityToColor(double velocity) {
        if (velocity > deadband)
            return green;
        else if (velocity < -deadband)
            return red;
        return blue;
    }

    private void setArrowAngle(double velocity, double targetVelocity) {
        if (velocity > deadband) {
            currentVelocityTopArrowLigament.setAngle(positiveTopAngle);
            currentVelocityBottomArrowLigament.setAngle(positiveBottomAngle);
        } else if (velocity < -deadband) {
            currentVelocityTopArrowLigament.setAngle(negativeTopAngle);
            currentVelocityBottomArrowLigament.setAngle(negativeBottomAngle);
        } else {
            currentVelocityTopArrowLigament.setAngle(zeroTopAngle);
            currentVelocityBottomArrowLigament.setAngle(zeroBottomAngle);
        }
        if (targetVelocity > deadband) {
            targetVelocityTopArrowLigament.setAngle(positiveTopAngle);
            targetVelocityBottomArrowLigament.setAngle(positiveBottomAngle);
        } else if (targetVelocity < -deadband) {
            targetVelocityTopArrowLigament.setAngle(negativeTopAngle);
            targetVelocityBottomArrowLigament.setAngle(negativeBottomAngle);
        } else {
            targetVelocityTopArrowLigament.setAngle(zeroTopAngle);
            targetVelocityBottomArrowLigament.setAngle(zeroBottomAngle);
        }
    }
}