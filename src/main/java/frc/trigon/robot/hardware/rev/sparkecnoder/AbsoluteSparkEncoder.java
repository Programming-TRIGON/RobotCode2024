package frc.trigon.robot.hardware.rev.sparkecnoder;

import com.revrobotics.SparkAbsoluteEncoder;

public class AbsoluteSparkEncoder extends SparkEncoder {
    private final SparkAbsoluteEncoder encoder;

    public AbsoluteSparkEncoder(SparkAbsoluteEncoder encoder) {
        this.encoder = encoder;
        setConversionsFactor(1);
    }

    public double getPositionRevolutions() {
        return encoder.getPosition();
    }

    public double getVelocityRevolutionsPerSecond() {
        return encoder.getVelocity();
    }

    @Override
    public void setConversionsFactor(double conversionsFactor) {
        encoder.setPositionConversionFactor(conversionsFactor);
        encoder.setVelocityConversionFactor(conversionsFactor / 60);
    }
}
