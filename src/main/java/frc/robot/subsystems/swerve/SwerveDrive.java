package frc.robot.subsystems.swerve;

import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModulePosition;

import com.ctre.phoenix6.configs.Pigeon2Configuration;
import com.ctre.phoenix6.hardware.Pigeon2;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.path.PathPlannerPath;
import com.pathplanner.lib.util.HolonomicPathFollowerConfig;
import com.pathplanner.lib.util.PIDConstants;
import com.pathplanner.lib.util.ReplanningConfig;

import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.vision.PoseEstimationSystem;

public class SwerveDrive extends SubsystemBase {
    public SwerveDrivePoseEstimator swerveOdometry;
    public SwerveModule[] mSwerveMods;
    public Pigeon2 gyro;

    private PoseEstimationSystem visionSubsystem = null;

    private final Translation2d RED_GOAL_POSE = new Translation2d(16.579342, 5.547867999999999);
    private final Translation2d BLUE_GOAL_POSE = new Translation2d(-0.038099999999999995, 5.547867999999999);
    
    private final Translation2d RED_GOAL_POSE_ROTATION_TARGET = new Translation2d(16.579342 - Units.inchesToMeters(9.1), 5.547867999999999);
    private final Translation2d BLUE_GOAL_POSE_ROTATION_TARGET = new Translation2d(-0.038099999999999995 + Units.inchesToMeters(9.1), 5.547867999999999);

    private Translation2d targetGoalPose;
    private Translation2d targetGoalPoseRotationTarget;

    Field2d field2d;

    public SwerveDrive() {

        field2d = new Field2d();
        SmartDashboard.putData(field2d);
        field2d.setRobotPose(new Pose2d(RED_GOAL_POSE, new Rotation2d()));

        gyro = new Pigeon2(SwerveConstants.pigeonID);
        gyro.getConfigurator().apply(new Pigeon2Configuration());
        gyro.setYaw(0);

        mSwerveMods = new SwerveModule[] {
                new SwerveModule(0, SwerveConstants.Mod0.constants),
                new SwerveModule(1, SwerveConstants.Mod1.constants),
                new SwerveModule(2, SwerveConstants.Mod2.constants),
                new SwerveModule(3, SwerveConstants.Mod3.constants)
        };

        Timer.delay(1.0);
        resetModulesToAbsolute();

        var stateStdDevs = VecBuilder.fill(0.1, 0.1, 0.1);
        var visionStdDevs = VecBuilder.fill(1, 1, 1);

        swerveOdometry = new SwerveDrivePoseEstimator(
                SwerveConstants.swerveKinematics,
                getGyroYaw(),
                getModulePositions(),
                new Pose2d(),
                stateStdDevs,
                visionStdDevs);

        AutoBuilder.configureHolonomic(
                this::getPose,
                this::setPose,
                this::getRobotRelativeSpeeds,
                this::driveRobotRelative,
                new HolonomicPathFollowerConfig(
                        new PIDConstants(SwerveConstants.AutoConstants.TRANSLATION_CONTROLLER_P_COEFF, 0.0, 0.0),
                        new PIDConstants(SwerveConstants.AutoConstants.ROTATION_CONTROLLER_P_COEFF, 0.0, 0.0),
                        SwerveConstants.maxSpeed,
                        Math.hypot(SwerveConstants.trackWidth / 2.0, SwerveConstants.wheelBase / 2.0),
                        new ReplanningConfig()),
                () -> {

                    var alliance = DriverStation.getAlliance();
                    if (alliance.isPresent()) {
                        return alliance.get() == DriverStation.Alliance.Red;
                    }
                    return false;
                },
                this);

        setTargetGoal();
    }

    public void setTargetGoal() {
        boolean isRed = DriverStation.getAlliance().isPresent() && DriverStation.getAlliance().get() == Alliance.Red;

        targetGoalPose = isRed
                ? RED_GOAL_POSE
                : BLUE_GOAL_POSE;

        targetGoalPoseRotationTarget = isRed
                ? RED_GOAL_POSE_ROTATION_TARGET
                : BLUE_GOAL_POSE_ROTATION_TARGET;


    }

    public SwerveDrive(PoseEstimationSystem visionSubsystem) {
        this();
        this.visionSubsystem = visionSubsystem;
    }

    public ChassisSpeeds getRobotRelativeSpeeds() {
        return SwerveConstants.swerveKinematics.toChassisSpeeds(getModuleStates());
    }

    public void driveRobotRelative(ChassisSpeeds speeds) {
        drive(new Translation2d(speeds.vxMetersPerSecond, speeds.vyMetersPerSecond), speeds.omegaRadiansPerSecond,
                false);
    }

    public void drive(Translation2d translation, double rotation, boolean fieldRelative) {
        SwerveModuleState[] swerveModuleStates = SwerveConstants.swerveKinematics.toSwerveModuleStates(
                fieldRelative ? ChassisSpeeds.fromFieldRelativeSpeeds(
                        translation.getX(),
                        translation.getY(),
                        rotation,
                        getHeading())
                        : new ChassisSpeeds(
                                translation.getX(),
                                translation.getY(),
                                rotation));
        SwerveDriveKinematics.desaturateWheelSpeeds(swerveModuleStates, SwerveConstants.maxSpeed);

        for (SwerveModule mod : mSwerveMods) {
            mod.setDesiredState(swerveModuleStates[mod.moduleNumber]);
        }
    }

    public SwerveModuleState[] getModuleStates() {
        SwerveModuleState[] states = new SwerveModuleState[4];
        for (SwerveModule mod : mSwerveMods) {
            states[mod.moduleNumber] = mod.getState();
        }
        return states;
    }

    public SwerveModulePosition[] getModulePositions() {
        SwerveModulePosition[] positions = new SwerveModulePosition[4];
        for (SwerveModule mod : mSwerveMods) {
            positions[mod.moduleNumber] = mod.getPosition();
        }
        return positions;
    }

    public Pose2d getPose() {
        return swerveOdometry.getEstimatedPosition();
    }

    public void setPose(Pose2d pose) {
        swerveOdometry.resetPosition(getGyroYaw(), getModulePositions(), pose);
    }

    public void addVisionMeasurement(Pose2d visionMeasurement, double timestamp) {
        swerveOdometry.addVisionMeasurement(visionMeasurement, timestamp);
    }

    public Rotation2d getHeading() {
        return getPose().getRotation();
    }

    public double getHeadingDegrees(){
        double heading = getHeading().getDegrees();
        if(heading < 0){
            heading += 360;
        }
        return heading;
    }

    public void setHeading(Rotation2d heading) {
        swerveOdometry.resetPosition(getGyroYaw(), getModulePositions(),
                new Pose2d(getPose().getTranslation(), heading));
    }

    public void zeroHeading() {
        swerveOdometry.resetPosition(getGyroYaw(), getModulePositions(),
                new Pose2d(getPose().getTranslation(), new Rotation2d(Math.toRadians(180))));
    }

    public Rotation2d getGyroYaw() {
        return Rotation2d.fromDegrees(gyro.getYaw().getValue());
    }

    public void resetModulesToAbsolute() {
        for (SwerveModule mod : mSwerveMods) {
            mod.resetToAbsolute();
        }
    }

    public Command followPathCommand(String pathName, boolean initialPath) {
        PathPlannerPath path = PathPlannerPath.fromPathFile(pathName);

        if (initialPath) {
            setPose(path.getPreviewStartingHolonomicPose());
        }

        return AutoBuilder.followPath(path);
    }

    private Translation2d getTargetGoalPose() {
        return targetGoalPose;
    }

    private Translation2d getTargetGoalPoseRotationTarget() {
        return targetGoalPoseRotationTarget;
    }

    // returns the distance, in meters, from the center of the robot to the target
    // goal
    public double getDistanceMetersToGoal() {
        return getTargetGoalPose().getDistance(swerveOdometry.getEstimatedPosition().getTranslation());
    }

    // returns the angle, in degrees, that the robot needs to be pointing in order
    // to be pointing at the target goal
    public double getAngleDegreesToGoal() {
        Translation2d goalPose = getTargetGoalPoseRotationTarget();
        Translation2d robotPose = getPose().getTranslation();
        Translation2d robotToGoal = goalPose.minus(robotPose);
        return Math.toDegrees(Math.atan2(robotToGoal.getY(), robotToGoal.getX())) + 180;
    }

    @Override
    public void periodic() {
        swerveOdometry.update(getGyroYaw(), getModulePositions());

        if (visionSubsystem != null) {
            if (visionSubsystem.getLeftVisionEstimatePresent()) {
                addVisionMeasurement(
                        new Pose2d(visionSubsystem.getLeftVisionEstimatePose2d().getTranslation(), getHeading()),
                        visionSubsystem.getLeftVisionEstimateTimestamp());
            }

            if (visionSubsystem.getRightVisionEstimatePresent()) {
                addVisionMeasurement(
                        new Pose2d(visionSubsystem.getRightVisionEstimatePose2d().getTranslation(), getHeading()),
                        visionSubsystem.getRightVisionEstimateTimestamp());
            }
        }
        field2d.setRobotPose(getPose());
        SmartDashboard.putNumber("distance", getDistanceMetersToGoal());
        SmartDashboard.putNumber("rotation angle to goal", getAngleDegreesToGoal());
        SmartDashboard.putNumber("heading", getHeadingDegrees());
        // System.out.println(getPose());
        // System.out.println("distance from target meters: " + getDistanceMetersToGoal());
        // System.out.println("distance from target: " + getDistanceMetersToGoal() + " angle to target: " + (getAngleDegreesToGoal()) + " robot heading: " + getHeadingDegrees());
    }
}