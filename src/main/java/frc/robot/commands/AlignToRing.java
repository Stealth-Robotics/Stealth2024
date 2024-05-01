package frc.robot.commands;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.swerve.SwerveDrive;
import frc.robot.util.BetterPID;

public class AlignToRing extends Command {

    private final SwerveDrive swerve;
    private final BetterPID rotationPID;

    // TODO: TUNE CONSTANTS
    private double kP = 0.075;
    private double kI = 0.01;
    private double kD = 0.05;

    private double kTolerance = 2.5;

    public AlignToRing(SwerveDrive swerve) {
        this.swerve = swerve;
        rotationPID = new BetterPID(kP, kI, kD, true);
        rotationPID.setTolerance(kTolerance);

        // will remove once tested
        // throw new UnsupportedOperationException("AutoAlignCommand is not yet
        // implemented");
    }

    @Override
    public void initialize() {
        rotationPID.setSetpoint(0);
    }

    @Override
    public void execute() {
        double rotationOutput = rotationPID.calculate(swerve.getTx());
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
}
