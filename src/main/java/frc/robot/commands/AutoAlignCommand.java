package frc.robot.commands;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.Command;
import frc.lib.util.BetterPID;
import frc.robot.subsystems.swerve.SwerveDrive;

public class AutoAlignCommand extends Command {

    private final SwerveDrive swerve;
    private final BetterPID rotationPID;

    // TODO: TUNE CONSTANTS
    private final double kP = 0.1;
    private final double kI = 0;
    private final double kD = 0;

    private double kTolerance = 0.1;

    // Poses pulled from tag IDs 4 and 7, depending on alliance from here:
    // https://github.com/wpilibsuite/allwpilib/blob/main/apriltag/src/main/native/resources/edu/wpi/first/apriltag/2024-crescendo.json

    public AutoAlignCommand(SwerveDrive swerve) {
        this.swerve = swerve;
        rotationPID = new BetterPID(kP, kI, kD);
        rotationPID.setTolerance(kTolerance);
        addRequirements(swerve);

        //will remove once tested
        throw new UnsupportedOperationException("AutoAlignCommand is not yet implemented");
    }

    @Override
    public void initialize() {
        rotationPID.setSetpoint(swerve.getAngleDegreesToGoal());
    }

    @Override
    public void execute() {
        double rotationOutput = rotationPID.calculate(swerve.getPose().getRotation().getDegrees());
        swerve.drive(new Translation2d(), rotationOutput, false);
    }

    @Override
    public boolean isFinished() {
        return rotationPID.atSetpoint();
    }

    @Override
    public void end(boolean interrupted) {
        swerve.drive(new Translation2d(), 0, false);
    }

    @Override
    public void initSendable(SendableBuilder builder) {
        super.initSendable(builder);
        builder.setSmartDashboardType("AutoAlign PID Gains");
        builder.addDoubleProperty("kP", rotationPID::getP, rotationPID::setP);
        builder.addDoubleProperty("kI", rotationPID::getI, rotationPID::setI);
        builder.addDoubleProperty("kD", rotationPID::getD, rotationPID::setD);
        builder.addDoubleProperty("kTolerance", () -> this.kTolerance, (value) -> {
            this.kTolerance = value;
            rotationPID.setTolerance(this.kTolerance);
        });
    }

}
