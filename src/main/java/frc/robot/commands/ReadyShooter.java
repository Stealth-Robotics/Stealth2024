package frc.robot.commands;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.subsystems.swerve.SwerveDrive;

public class ReadyShooter extends Command{

    private final ShooterSubsystem shooterSubsystem;
    private final Translation2d robotPosition;
    private final Translation2d targetPosition;

    public ReadyShooter(ShooterSubsystem shooterSubsystem, SwerveDrive drive){
        this.shooterSubsystem = shooterSubsystem;
        this.robotPosition = drive.getPose().getTranslation();
        this.targetPosition = drive.get
    }

    
}
