package frc.robot.commands;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.swerve.SwerveConstants;
import frc.robot.subsystems.swerve.SwerveDrive;
import frc.robot.util.BetterPID;
import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;

public class DriveWhileAlignedToSpeaker extends Command {
    private SwerveDrive swerveSubsystem;
    private DoubleSupplier translationSup;
    private DoubleSupplier strafeSup;
    private BooleanSupplier isRed;

    private double kP = 0.1;
    private double kI = 0.2;
    private double kD = 0.0;

    private double kTolerance = 0.5;

    private final BetterPID rotationPID;

    public DriveWhileAlignedToSpeaker(SwerveDrive swerveSubsystem, DoubleSupplier translationSup,
            DoubleSupplier strafeSup, BooleanSupplier isRed) {
        this.swerveSubsystem = swerveSubsystem;

        this.translationSup = translationSup;
        this.strafeSup = strafeSup;
        this.isRed = isRed;

        rotationPID = new BetterPID(kP, kI, kD, true);
        rotationPID.setTolerance(kTolerance);
        rotationPID.enableContinuousInput(0, 360);
        addRequirements(swerveSubsystem);
    }

    @Override
    public void execute() {

        /* Get Values, Deadband */
        double translationVal = MathUtil.applyDeadband(translationSup.getAsDouble(), SwerveConstants.stickDeadband);
        double strafeVal = MathUtil.applyDeadband(strafeSup.getAsDouble(), SwerveConstants.stickDeadband);
        rotationPID.setSetpoint(swerveSubsystem.getAngleDegreesToGoal());
        double rotationVal = rotationPID.calculate(swerveSubsystem.getHeadingDegrees());

        if (isRed.getAsBoolean()) {
            translationVal *= -1;
            strafeVal *= -1;
        }

        /* Drive */
        swerveSubsystem.drive(new Translation2d(translationVal, strafeVal).times(SwerveConstants.maxSpeed), rotationVal,
                true);
    }
}
