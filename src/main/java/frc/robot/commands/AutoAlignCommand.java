package frc.robot.commands;

import com.pathplanner.lib.util.PIDConstants;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.swerve.SwerveDrive;
import frc.robot.subsystems.vision.LimelightHelpers;
import frc.robot.util.BetterPID;

public class AutoAlignCommand extends Command {

    private final SwerveDrive swerve;
    private final PIDController rotationPID;

    // TODO: TUNE CONSTANTS
    private double kP = 0.05;
    private double kI = 0.0;
    private double kD = 0.0;

    private double kTolerance = 2;

    public AutoAlignCommand(SwerveDrive swerve) {
        this.swerve = swerve;
        rotationPID = new PIDController(kP, kI, kD);
        rotationPID.setTolerance(kTolerance);
        addRequirements(swerve);

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
        double rotationOutput = rotationPID.calculate(LimelightHelpers.getTX("limelight-main"));
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
