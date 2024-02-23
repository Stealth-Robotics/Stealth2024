package frc.robot.subsystems.shooter;

import edu.wpi.first.math.interpolation.InterpolatingDoubleTreeMap;

public class DistanceToShotValuesMap {
    private static final InterpolatingDoubleTreeMap SHOOTER_INTERPOLATION_MAP = new InterpolatingDoubleTreeMap();
    private static final InterpolatingDoubleTreeMap ROTATION_INTERPOLATION_MAP = new InterpolatingDoubleTreeMap();

    public DistanceToShotValuesMap()
    {
        // TODO: Add values
        SHOOTER_INTERPOLATION_MAP.put(0.0, 0.0);

        // TODO: Add values
        //values should be in degrees
        ROTATION_INTERPOLATION_MAP.put(0.0, 0.0);
    }

    public static double getInterpolatedShooterSpeed(double distance) {
        return SHOOTER_INTERPOLATION_MAP.get(distance);
    }

    /**
     * Interpolates the rotation angle based on the distance to the goal
     * @param distance in feet
     * returns interpolated rotation angle in degrees
     * @return interpolated rotation angle in degrees
     */
    public static double getInterpolatedRotationAngle(double distance) {
        return ROTATION_INTERPOLATION_MAP.get(distance);
    }
    
}
