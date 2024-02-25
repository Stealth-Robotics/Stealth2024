package frc.robot.subsystems.shooter;

import edu.wpi.first.math.interpolation.InterpolatingDoubleTreeMap;
import edu.wpi.first.math.util.Units;

public class DistanceToShotValuesMap {
    private final InterpolatingDoubleTreeMap SHOOTER_INTERPOLATION_MAP = new InterpolatingDoubleTreeMap();
    private final InterpolatingDoubleTreeMap ROTATION_INTERPOLATION_MAP = new InterpolatingDoubleTreeMap();

    public DistanceToShotValuesMap()
    {
        // TODO: Add values
        SHOOTER_INTERPOLATION_MAP.put(1.47, 92.0);
        SHOOTER_INTERPOLATION_MAP.put(3.0, 92.0);

        SHOOTER_INTERPOLATION_MAP.put(4.2, 92.0);
        SHOOTER_INTERPOLATION_MAP.put(6.1, 92.0);


        // TODO: Add values
        //values should be in rotations
        ROTATION_INTERPOLATION_MAP.put(1.3, Units.degreesToRotations(27.0));
        ROTATION_INTERPOLATION_MAP.put(1.4, Units.degreesToRotations(27.0));
        ROTATION_INTERPOLATION_MAP.put(1.5, Units.degreesToRotations(27.0));
        ROTATION_INTERPOLATION_MAP.put(3.0, Units.degreesToRotations(42.5));
        ROTATION_INTERPOLATION_MAP.put(4.2, Units.degreesToRotations(49.5));
        ROTATION_INTERPOLATION_MAP.put(6.1, Units.degreesToRotations(56));
    }

    public double getInterpolatedShooterSpeed(double distance) {
        return SHOOTER_INTERPOLATION_MAP.get(distance);
    }

    /**
     * Interpolates the rotation angle based on the distance to the goal
     * @param distance in feet
     * returns interpolated rotation angle in degrees
     * @return interpolated rotation angle in degrees
     */
    public double getInterpolatedRotationAngle(double distance) {
        return ROTATION_INTERPOLATION_MAP.get(distance);
    }
    
}
