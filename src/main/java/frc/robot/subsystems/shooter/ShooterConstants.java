package frc.robot.subsystems.shooter;

import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

public final class ShooterConstants {

    public static ShooterConfig config = new ShooterConfig(
        16,
        17,

        NeutralModeValue.Coast,
        InvertedValue.Clockwise_Positive
    );

    public static ShooterGains gains = new ShooterGains(
        0,
        0,

        0,
        0,
        0,
        
        0,
        0,
        0
    );

    // Records

    public record ShooterConfig(
        int leftMotorID,
        int rightMotorID,

        NeutralModeValue neutralMode,
        InvertedValue inverted
    ) {}

    public record ShooterGains(
        double MOTION_MAGIC_ACCELERATION,
        double MOTION_MAGIC_JERK,

        double kS,
        double kV,
        double kA,

        double kP,
        double kI,
        double kD
    ) {}
}
