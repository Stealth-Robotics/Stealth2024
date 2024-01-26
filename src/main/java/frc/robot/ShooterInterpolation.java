package frc.robot;

import edu.wpi.first.math.interpolation.InterpolatingDoubleTreeMap;

public class ShooterInterpolation {
    private static final InterpolatingDoubleTreeMap SHOOTER_INTERPOLATION_MAP = new InterpolatingDoubleTreeMap();
    private static final InterpolatingDoubleTreeMap ROTATION_INTERPOLATION_MAP = new InterpolatingDoubleTreeMap();

    //TODO: Actually find these
    static {
        SHOOTER_INTERPOLATION_MAP.put(0.0, 0.0);
        ROTATION_INTERPOLATION_MAP.put(0.0, 0.0);
        
    }

    public static double getInterpolatedShooterSpeed(double distance) {
        return SHOOTER_INTERPOLATION_MAP.get(distance);
    }

    public static double getInterpolatedRotationSpeed(double distance) {
        return ROTATION_INTERPOLATION_MAP.get(distance);
    }
    
}
