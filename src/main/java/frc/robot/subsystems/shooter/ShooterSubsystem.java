package frc.robot.subsystems.shooter;

import static frc.robot.subsystems.shooter.ShooterConstants.gains;

import java.util.function.DoubleSupplier;

import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.networktables.LoggedDashboardNumber;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.util.Alert;
import frc.robot.util.LoggedTunableNumber;
import frc.robot.util.Alert.AlertType;

public class ShooterSubsystem extends SubsystemBase {

    private static LoggedTunableNumber kP = new LoggedTunableNumber("Shooter/kP", gains.kP());
    private static LoggedTunableNumber kI = new LoggedTunableNumber("Shooter/kI", gains.kI());
    private static LoggedTunableNumber kD = new LoggedTunableNumber("Shooter/kD", gains.kD());

    private static LoggedTunableNumber kS = new LoggedTunableNumber("Shooter/kS", gains.kS());
    private static LoggedTunableNumber kV = new LoggedTunableNumber("Shooter/kV", gains.kV());
    private static LoggedTunableNumber kA = new LoggedTunableNumber("Shooter/kA", gains.kA());

    private static LoggedTunableNumber MOTION_MAGIC_ACCELERATION = new LoggedTunableNumber("Shooter/MOTION_MAGIC_ACCELERATION", gains.MOTION_MAGIC_ACCELERATION());
    private static LoggedTunableNumber MOTION_MAGIC_JERK = new LoggedTunableNumber("Shooter/MOTION_MAGIC_JERK", gains.MOTION_MAGIC_JERK());

    private static LoggedDashboardNumber leftShooterVelocity = new LoggedDashboardNumber("Shooter/LeftShooterVelocity", 0.0);
    private static LoggedDashboardNumber rightShooterVelocity = new LoggedDashboardNumber("Shooter/RightShooterVelocity", 0.0);

    private final Alert leftMotorDisconnectedAlert = new Alert("Left Shooter Motor Disconnected", AlertType.WARNING);
    private final Alert rightMotorDisconnectedAlert = new Alert("Right Shooter Motor Disconnected", AlertType.WARNING);

    ShooterIO shooterIO;
    ShooterIOInputsAutoLogged inputs = new ShooterIOInputsAutoLogged();

    public ShooterSubsystem(ShooterIO shooterIO) {
        this.shooterIO = shooterIO;

        switch (Constants.currentMode) {
            case REAL:
                break;
            case SIM:
                shooterIO.setPID(0, 0, 0);
            default:
                break;
        }
    }

    @Override
    public void periodic() {
        shooterIO.updateInputs(inputs);
        Logger.processInputs("Shooter", inputs);
    }

    public void setLeftVolts(double volts) {
        shooterIO.setLeftShooterVolts(volts);
    }

    public void setRightVolts(double volts) {
        shooterIO.setRightShooterVolts(volts);
    }

    public void setLeftVelocity(double velocity) {
        shooterIO.setLeftShooterVelocity(velocity);

        Logger.recordOutput("Shooter/LeftShooterVelocitySetpoint", velocity);
    }

    public void setRightVelocity(double velocity) {
        shooterIO.setRightShooterVelocity(velocity);

        Logger.recordOutput("Shooter/RightShooterVelocitySetpoint", velocity);
    }

    public void stop() {
        shooterIO.stopShooter();
    }

    @AutoLogOutput
    public double getLeftVelocity() {
        return inputs.leftShooterVelocity / 60.0;
    }

    @AutoLogOutput
    public double getRightVelocity() {
        return inputs.rightShooterVelocity / 60.0;
    }
}
