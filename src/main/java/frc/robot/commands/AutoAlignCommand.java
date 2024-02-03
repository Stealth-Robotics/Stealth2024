package frc.robot.commands;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.swerve.SwerveDrive;
import frc.robot.util.BetterPID;

public class AutoAlignCommand extends Command {

    private final SwerveDrive swerve;
    private final BetterPID rotationPID;

    // TODO: TUNE CONSTANTS
    private double kP = 0.1;
    private double kI = 0;
    private double kD = 0;

    private double kTolerance = 0.1;

    public AutoAlignCommand(SwerveDrive swerve) {
        this.swerve = swerve;
        rotationPID = new BetterPID(kP, kI, kD);
        rotationPID.setTolerance(kTolerance);
        addRequirements(swerve);

        // will remove once tested
        throw new UnsupportedOperationException("AutoAlignCommand is not yet implemented");
    }

    private void applyGains() {
        rotationPID.setP(kP);
        rotationPID.setI(kI);
        rotationPID.setD(kD);
        rotationPID.setTolerance(kTolerance);
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
        builder.addDoubleProperty("kP", () -> this.kP, (value) -> {
            this.kP = value;
            applyGains();
        });
        builder.addDoubleProperty("kI", () -> this.kI, (value) -> {
            this.kI = value;
            applyGains();
        });
        builder.addDoubleProperty("kD", () -> this.kD, (value) -> {
            this.kD = value;
            applyGains();
        });
        builder.addDoubleProperty("kTolerance", () -> this.kTolerance, (value) -> {
            this.kTolerance = value;
            applyGains();
        });
    }

}
