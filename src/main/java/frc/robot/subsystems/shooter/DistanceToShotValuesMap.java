package frc.robot.subsystems.shooter;

import edu.wpi.first.math.interpolation.InterpolatingDoubleTreeMap;
import edu.wpi.first.math.util.Units;

public class DistanceToShotValuesMap {
    private final InterpolatingDoubleTreeMap SHOOTER_INTERPOLATION_MAP = new InterpolatingDoubleTreeMap();
    private final InterpolatingDoubleTreeMap ROTATION_INTERPOLATION_MAP = new InterpolatingDoubleTreeMap();

    public DistanceToShotValuesMap()
    {
        // TODO: Add values
        SHOOTER_INTERPOLATION_MAP.put(0.0, 90.0);
        SHOOTER_INTERPOLATION_MAP.put(1.4, 90.0);
        SHOOTER_INTERPOLATION_MAP.put(3.0, 80.0);

        
        SHOOTER_INTERPOLATION_MAP.put(4.2, 90.0);
        SHOOTER_INTERPOLATION_MAP.put(6.1, 90.0);


        // TODO: Add values
        //values should be in rotations
        ROTATION_INTERPOLATION_MAP.put(0.0, Units.degreesToRotations(27.0));
        ROTATION_INTERPOLATION_MAP.put(1.3, Units.degreesToRotations(27.0));
        ROTATION_INTERPOLATION_MAP.put(1.4, Units.degreesToRotations(27.0));
        ROTATION_INTERPOLATION_MAP.put(1.5, Units.degreesToRotations(30.0));
        ROTATION_INTERPOLATION_MAP.put(1.9, Units.degreesToRotations(37.0));
        ROTATION_INTERPOLATION_MAP.put(2.2, Units.degreesToRotations(39));
        ROTATION_INTERPOLATION_MAP.put(2.4, Units.degreesToRotations(43));

        ROTATION_INTERPOLATION_MAP.put(3.0, Units.degreesToRotations(46.0));
        ROTATION_INTERPOLATION_MAP.put(3.4, Units.degreesToRotations(47.0));
        
        ROTATION_INTERPOLATION_MAP.put(3.8, Units.degreesToRotations(49));
        ROTATION_INTERPOLATION_MAP.put(3.99, Units.degreesToRotations(51.5));
        ROTATION_INTERPOLATION_MAP.put(4.3, Units.degreesToRotations(51.5));
        ROTATION_INTERPOLATION_MAP.put(4.6, Units.degreesToRotations(51.5));
        ROTATION_INTERPOLATION_MAP.put(5.3, Units.degreesToRotations(52));
        
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
