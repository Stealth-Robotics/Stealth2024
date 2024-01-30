package frc.robot.commands;

import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.shooter.ShooterInterpolation;
import frc.robot.subsystems.shooter.ShooterSubsystem;
import frc.robot.subsystems.swerve.SwerveDrive;

public class ReadyShooter extends Command{

    private final ShooterSubsystem shooterSubsystem;
    private final SwerveDrive drive;
    private final double SPIN_MULTIPLIER = 0.8;

    public ReadyShooter(ShooterSubsystem shooterSubsystem, SwerveDrive drive){
        this.shooterSubsystem = shooterSubsystem;
        this.drive = drive;
        addRequirements(shooterSubsystem);
    }

    @Override
    public void initialize() {
        // gets distance in feet because that's what the interpolation map uses
        double distance = Units.metersToFeet(drive.getDistanceMetersToGoal());
        double targetSpeed = ShooterInterpolation.getInterpolatedShooterSpeed(distance);

        shooterSubsystem.setRightVelocity(targetSpeed);
        shooterSubsystem.setLeftVelocity(targetSpeed * SPIN_MULTIPLIER);
    }

    @Override
    public boolean isFinished() {
        return shooterSubsystem.motorsAtTargetVelocity();
    }
    
}
