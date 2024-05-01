package frc.robot.subsystems.rotator;

import org.littletonrobotics.junction.AutoLog;

public interface RotatorIO {

    @AutoLog
    public static class RotatorInputs {
        double rotatorAngleRotations;
        double rotatorAppliedVolts;
        double currentDrawAmps;
        double angVeloRadPerSec;
        double rotatorGoalPositionRotations;
    }
    
    default void updateInputs(RotatorInputs inputs){}
    default void setTargetAngle(double rotations){}
    
}
