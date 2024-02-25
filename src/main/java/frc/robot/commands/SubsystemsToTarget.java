package frc.robot.commands;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.RotatorSubsystem;
import frc.robot.subsystems.shooter.DistanceToShotValuesMap;
import frc.robot.subsystems.shooter.ShooterSubsystem;
import frc.robot.subsystems.swerve.SwerveDrive;

public class SubsystemsToTarget extends Command{

    private final RotatorSubsystem rotator;
    private final ShooterSubsystem shooter;
    private final SwerveDrive drive;
    DistanceToShotValuesMap map;

    public SubsystemsToTarget(RotatorSubsystem rotator, ShooterSubsystem shooter, SwerveDrive drive, DistanceToShotValuesMap map){

        this.rotator = rotator;
        this.shooter = shooter;
        this.drive = drive;
        this.map = map;
        addRequirements(shooter, rotator);


    }

    @Override
    public void initialize() {
        rotator.setMotorTargetPosition(map.getInterpolatedRotationAngle(drive.getDistanceMetersToGoal()));
        shooter.setLeftVelocity(map.getInterpolatedShooterSpeed(drive.getDistanceMetersToGoal()));
        shooter.setRightVelocity(map.getInterpolatedShooterSpeed(drive.getDistanceMetersToGoal()) * shooter.SPIN_CONSTANT);
        System.out.println(drive.getDistanceMetersToGoal());

    }

    @Override
    public boolean isFinished() {
        return shooter.motorsAtTargetVelocity() && rotator.isMotorAtTarget();
    }
    
}
