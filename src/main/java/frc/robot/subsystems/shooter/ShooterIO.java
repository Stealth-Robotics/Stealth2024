package frc.robot.subsystems.shooter;

import org.littletonrobotics.junction.AutoLog;

public interface ShooterIO {

    @AutoLog
    class ShooterIOInputs {
        public boolean leftShooterConnected = true;
        public boolean rightShooterConnected = true;

        public double leftShooterVelocity = 0.0;
        public double leftShooterAppliedVolts = 0.0;
        public double leftShooterOutputCurrent = 0.0;
        public double leftShooterTempCelsius = 0.0;

        public double rightShooterVelocity = 0.0;
        public double rightShooterAppliedVolts = 0.0;
        public double rightShooterOutputCurrent = 0.0;
        public double rightShooterTempCelsius = 0.0;
    }

    default void updateInputs(ShooterIOInputs inputs) {
    }

    default void setPID(double kP, double kI, double kD) {
    }

    default void setFF(double kS, double kV, double kA) {
    }

    default void setLeftShooterVolts(double volts) {
    }

    default void setLeftShooterVelocity(double velocity) {
    }

    default void setRightShooterVolts(double volts) {
    }

    default void setRightShooterVelocity(double velocity) {
    }

    default void stopShooter() {
    }
}
