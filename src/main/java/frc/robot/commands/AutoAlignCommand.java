package frc.robot.commands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.PIDCommand;
import frc.robot.subsystems.swerve.SwerveDrive;
import frc.robot.util.BetterPID;

public class AutoAlignCommand extends Command {

    private final SwerveDrive swerve;
    private final BetterPID rotationPID;

    // TODO: TUNE CONSTANTS
    private double kP = 0.15;
    private double kI = 0.2;
    private double kD = 0.02;

    private double kTolerance = 0.5;

    public AutoAlignCommand(SwerveDrive swerve) {
        this.swerve = swerve;
        rotationPID = new BetterPID(kP, kI, kD, true);
        rotationPID.setTolerance(kTolerance);
        rotationPID.enableContinuousInput(0, 360);
        addRequirements(swerve);

        // will remove once tested
        // throw new UnsupportedOperationException("AutoAlignCommand is not yet implemented");
    }

    @Override
    public void initialize() {
        rotationPID.setSetpoint(swerve.getAngleDegreesToGoal());
    }

    @Override
    public void execute() {
        double rotationOutput = rotationPID.calculate(swerve.getHeadingDegrees());
        swerve.drive(new Translation2d(), rotationOutput, true);
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
        builder.addDoubleProperty("kTolerance", rotationPID::getPositionTolerance, rotationPID::setTolerance);
    }

}
