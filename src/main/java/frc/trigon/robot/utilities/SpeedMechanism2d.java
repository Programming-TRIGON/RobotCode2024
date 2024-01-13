package frc.trigon.robot.utilities;

import edu.wpi.first.wpilibj.smartdashboard.Mechanism2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismLigament2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismRoot2d;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj.util.Color8Bit;
import org.littletonrobotics.junction.Logger;

/**
 * A mechanism that displays the current velocity and target velocity.
 */
public class SpeedMechanism2d {
    private static final Color8Bit
            GREEN = new Color8Bit(Color.kGreen),
            RED = new Color8Bit(Color.kRed),
            BLUE = new Color8Bit(Color.kBlue);
    private static final double
            NEGATIVE_TOP_ANGLE = 45,
            NEGATIVE_BOTTOM_ANGLE = 315,
            POSITIVE_TOP_ANGLE = 210,
            POSITIVE_BOTTOM_ANGLE = 145,
            ZERO_TOP_ANGLE = 90,
            ZERO_BOTTOM_ANGLE = 270;
    private final String key;
    private final Mechanism2d mechanism;
    private final MechanismLigament2d
            currentVelocityLigament,
            currentVelocityTopArrowLigament,
            currentVelocityBottomArrowLigament,
            targetVelocityLigament,
            targetVelocityTopArrowLigament,
            targetVelocityBottomArrowLigament;
    private final double deadband;

    public SpeedMechanism2d(String key, double maxDisplayableVelocity) {
        this(key, maxDisplayableVelocity, 0.001);
    }

    public SpeedMechanism2d(String key, double maxDisplayableVelocity, double deadband) {
        this.deadband = deadband;
        this.key = key;
        this.mechanism = new Mechanism2d(2 * maxDisplayableVelocity, 2 * maxDisplayableVelocity);
        MechanismRoot2d root = mechanism.getRoot("VelocityRoot", maxDisplayableVelocity, maxDisplayableVelocity);
        this.currentVelocityLigament = root.append(new MechanismLigament2d("ZCurrentVelocityLigament", 0, 0, 5, new Color8Bit(Color.kBlue)));
        this.currentVelocityTopArrowLigament = currentVelocityLigament.append(new MechanismLigament2d(  "ZCurrentVelocityTopArrowLigament", 0.2 * maxDisplayableVelocity, ZERO_TOP_ANGLE, 5, new Color8Bit(Color.kBlue)));
        this.currentVelocityBottomArrowLigament = currentVelocityLigament.append(new MechanismLigament2d("ZCurrentVelocityBottomArrowLigament", 0.2 * maxDisplayableVelocity, ZERO_BOTTOM_ANGLE, 5, new Color8Bit(Color.kBlue)));

        this.targetVelocityLigament = root.append(new MechanismLigament2d("TargetVelocityLigament", 0, 0, 5, new Color8Bit(Color.kGray)));
        this.targetVelocityTopArrowLigament = targetVelocityLigament.append(new MechanismLigament2d("TargetVelocityTopArrowLigament", 0.2 * maxDisplayableVelocity, ZERO_TOP_ANGLE, 5, new Color8Bit(Color.kGray)));
        this.targetVelocityBottomArrowLigament = targetVelocityLigament.append(new MechanismLigament2d("TargetVelocityBottomArrowLigament", 0.2 * maxDisplayableVelocity, ZERO_BOTTOM_ANGLE, 5, new Color8Bit(Color.kGray)));
    }

    /**
     * Updates the mechanism's velocity and target velocity.
     * @param velocity the current velocity
     * @param targetVelocity the target velocity
     */
    public void updateMechanism(double velocity, double targetVelocity) {
        setTargetVelocity(targetVelocity);
        updateMechanism(velocity);
    }

    /**
     * Updates the mechanism's velocity.
     * @param velocity the current velocity
     */
    public void updateMechanism(double velocity) {
        setArrowAngle(velocity, currentVelocityTopArrowLigament, currentVelocityBottomArrowLigament);
        currentVelocityLigament.setLength(velocity);
        setCurrentLigamentColor(velocityToColor(velocity));
        Logger.recordOutput(key, mechanism);
    }

    public void setTargetVelocity(double targetVelocity) {
        setArrowAngle(targetVelocity, targetVelocityTopArrowLigament, targetVelocityBottomArrowLigament);
        targetVelocityLigament.setLength(targetVelocity);
    }

    private void setCurrentLigamentColor(Color8Bit color) {
        currentVelocityLigament.setColor(color);
        currentVelocityTopArrowLigament.setColor(color);
        currentVelocityBottomArrowLigament.setColor(color);
    }

    private Color8Bit velocityToColor(double velocity) {
        if (velocity > deadband)
            return GREEN;
        else if (velocity < -deadband)
            return RED;
        return BLUE;
    }

    private void setArrowAngle(double velocity, MechanismLigament2d topLigament, MechanismLigament2d bottomLigament) {
        if (velocity > deadband) {
            topLigament.setAngle(POSITIVE_TOP_ANGLE);
            bottomLigament.setAngle(POSITIVE_BOTTOM_ANGLE);
        } else if (velocity < -deadband) {
            topLigament.setAngle(NEGATIVE_TOP_ANGLE);
            bottomLigament.setAngle(NEGATIVE_BOTTOM_ANGLE);
        } else {
            topLigament.setAngle(ZERO_TOP_ANGLE);
            bottomLigament.setAngle(ZERO_BOTTOM_ANGLE);
        }
    }
}