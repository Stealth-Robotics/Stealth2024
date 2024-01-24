package frc.robot.commands;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.Command;
import frc.lib.util.BetterPID;
import frc.robot.subsystems.swerve.SwerveDrive;

public class AutoAlignCommand extends Command {

    private final SwerveDrive swerve;
    private final BetterPID rotationPID;

    private Pose2d currentPose2d;
    // TODO: TUNE CONSTANTS
    private final double kP = 0.1;
    private final double kI = 0;
    private final double kD = 0;

    private final double kTolerance = 0.1;

    Translation2d RED_GOAL_POSE = new Translation2d(16.579342, 5.547867999999999);
    Translation2d BLUE_GOAL_POSE = new Translation2d(-0.038099999999999995, 5.547867999999999);

    public AutoAlignCommand(SwerveDrive swerve) {
        this.swerve = swerve;
        rotationPID = new BetterPID(kP, kI, kD);
        rotationPID.setTolerance(kTolerance);
        addRequirements(swerve);
    }

    @Override
    public void initialize() {
        swerve.setPose(new Pose2d(5, BLUE_GOAL_POSE.getY(), Rotation2d.fromDegrees(90)));
        currentPose2d = swerve.getPose();

        // Poses pulled from tag IDs 4 and 7, depending on alliance from here:
        // https://github.com/wpilibsuite/allwpilib/blob/main/apriltag/src/main/native/resources/edu/wpi/first/apriltag/2024-crescendo.json
        boolean isRed = DriverStation.getAlliance().isPresent() && DriverStation.getAlliance().get() == Alliance.Red;

        Translation2d targetGoalPose = isRed
                ? RED_GOAL_POSE
                : BLUE_GOAL_POSE;

        Translation2d targetPoseOffset = targetGoalPose.minus(currentPose2d.getTranslation());

        double rotationGoal = Math.toDegrees(Math.atan2(targetPoseOffset.getY(), targetPoseOffset.getX()));

        rotationPID.setSetpoint(rotationGoal);
    }

    @Override
    public void execute() {
        double rotationOutput = MathUtil.clamp(
                rotationPID.calculate(swerve.getPose().getRotation().getDegrees()),
                -0.5,
                0.5);
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

}
