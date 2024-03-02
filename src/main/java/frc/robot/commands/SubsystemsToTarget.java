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
    private double distance = -1;

    public SubsystemsToTarget(RotatorSubsystem rotator, ShooterSubsystem shooter, SwerveDrive drive, DistanceToShotValuesMap map){

        this.rotator = rotator;
        this.shooter = shooter;
        this.drive = drive;
        this.map = map;
        addRequirements(shooter, rotator);


    }

    public SubsystemsToTarget(RotatorSubsystem rotator, ShooterSubsystem shooter, SwerveDrive drive, DistanceToShotValuesMap map, double distance){

        this.rotator = rotator;
        this.shooter = shooter;
        this.drive = drive;
        this.map = map;
        this.distance = distance;
        addRequirements(shooter, rotator);


    }

    @Override
    public void initialize() {
        double shooterSpeed;
        double rotationAngle;
        if(distance == -1){
            shooterSpeed = map.getInterpolatedShooterSpeed(drive.getDistanceMetersToGoal());
            rotationAngle = map.getInterpolatedRotationAngle(drive.getDistanceMetersToGoal());
        }
        else{
            shooterSpeed = map.getInterpolatedShooterSpeed(distance);
            rotationAngle = map.getInterpolatedRotationAngle(distance);
        }
        rotator.setMotorTargetPosition(rotationAngle);
        shooter.setLeftVelocity(shooterSpeed);
        shooter.setRightVelocity(shooterSpeed * shooter.SPIN_CONSTANT);
        System.out.println(drive.getDistanceMetersToGoal());

    }

    @Override
    public boolean isFinished() {
        return shooter.motorsAtTargetVelocity() && rotator.isMotorAtTarget();
    }
    
}
