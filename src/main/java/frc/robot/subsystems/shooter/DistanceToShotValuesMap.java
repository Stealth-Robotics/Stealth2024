package frc.robot.subsystems.shooter;

import edu.wpi.first.math.interpolation.InterpolatingDoubleTreeMap;
import edu.wpi.first.math.util.Units;

public class DistanceToShotValuesMap {
    private final InterpolatingDoubleTreeMap SHOOTER_INTERPOLATION_MAP = new InterpolatingDoubleTreeMap();
    private final InterpolatingDoubleTreeMap ROTATION_INTERPOLATION_MAP = new InterpolatingDoubleTreeMap();

    public DistanceToShotValuesMap()
    {
        // TODO: Add values
        SHOOTER_INTERPOLATION_MAP.put(0.0, 10.0);

        SHOOTER_INTERPOLATION_MAP.put(4.0, 93.0);
        SHOOTER_INTERPOLATION_MAP.put(5.0, 10.0);


        // TODO: Add values
        //values should be in rotations
        ROTATION_INTERPOLATION_MAP.put(0.0, Units.degreesToRotations(50));
        ROTATION_INTERPOLATION_MAP.put(4.0, Units.degreesToRotations(60));
        ROTATION_INTERPOLATION_MAP.put(5.0, Units.degreesToRotations(50));
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
