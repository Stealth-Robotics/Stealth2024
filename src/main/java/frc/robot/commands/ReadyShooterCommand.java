package frc.robot.commands;

import java.util.function.DoubleSupplier;

import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.shooter.ShooterInterpolationMaps;
import frc.robot.subsystems.shooter.ShooterSubsystem;
import frc.robot.subsystems.swerve.SwerveDrive;

public class ReadyShooterCommand extends Command {

    private final ShooterSubsystem shooterSubsystem;
    private final SwerveDrive drive;
    private final double SPIN_MULTIPLIER = 0.8;

    // allows us to modify the distance the robot thinks it is from the goal in
    // order to live offset the shooter speed/angle
    // will use some sort of slider or other thing, either virtual on dashboard
    // screen or a physical slider on the driverstation
    private final DoubleSupplier distanceOffset;

    public ReadyShooterCommand(ShooterSubsystem shooterSubsystem, SwerveDrive drive, DoubleSupplier distanceOffset) {
        this.shooterSubsystem = shooterSubsystem;
        this.drive = drive;
        this.distanceOffset = distanceOffset;
        addRequirements(shooterSubsystem);
    }

    public ReadyShooterCommand(ShooterSubsystem shooterSubsystem, SwerveDrive drive) {
        this(shooterSubsystem, drive, () -> 0.0);
    }

    @Override
    public void initialize() {
        // gets distance in feet because that's what the interpolation map uses
        double distance = Units.metersToFeet(drive.getDistanceMetersToGoal()) + distanceOffset.getAsDouble();
        double targetSpeed = ShooterInterpolationMaps.getInterpolatedShooterSpeed(distance);

        shooterSubsystem.setRightVelocity(targetSpeed);
        shooterSubsystem.setLeftVelocity(targetSpeed * SPIN_MULTIPLIER);
    }

    @Override
    public boolean isFinished() {
        return shooterSubsystem.motorsAtTargetVelocity();
    }

}
