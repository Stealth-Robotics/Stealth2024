package frc.robot.subsystems.shooter;

import edu.wpi.first.math.interpolation.InterpolatingDoubleTreeMap;

public class ShooterInterpolationMaps {
    private static final InterpolatingDoubleTreeMap SHOOTER_INTERPOLATION_MAP = new InterpolatingDoubleTreeMap();
    private static final InterpolatingDoubleTreeMap ROTATION_INTERPOLATION_MAP = new InterpolatingDoubleTreeMap();

    public ShooterInterpolationMaps()
    {
        // TODO: Add values
        SHOOTER_INTERPOLATION_MAP.put(0.0, 0.0);

        // TODO: Add values
        ROTATION_INTERPOLATION_MAP.put(0.0, 0.0);
    }

    public static double getInterpolatedShooterSpeed(double distance) {
        return SHOOTER_INTERPOLATION_MAP.get(distance);
    }

    public static double getInterpolatedRotationSpeed(double distance) {
        return ROTATION_INTERPOLATION_MAP.get(distance);
    }
    
}
