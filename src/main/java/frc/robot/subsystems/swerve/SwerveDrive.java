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

import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.vision.PoseEstimationSystem;

public class SwerveDrive extends SubsystemBase {
    public SwerveDrivePoseEstimator swerveOdometry;
    public SwerveModule[] mSwerveMods;
    public Pigeon2 gyro;

    private PoseEstimationSystem visionSubsystem = null;

    Translation2d RED_GOAL_POSE = new Translation2d(16.579342, 5.547867999999999);
    Translation2d BLUE_GOAL_POSE = new Translation2d(-0.038099999999999995, 5.547867999999999);

    Translation2d targetGoalPose;

    public SwerveDrive(PoseEstimationSystem visionSubsystem) {
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

        swerveOdometry = new SwerveDrivePoseEstimator(
                SwerveConstants.swerveKinematics,
                getGyroYaw(),
                getModulePositions(),
                new Pose2d());

        AutoBuilder.configureHolonomic(
                this::getPose,
                this::setPose,
                this::getRobotRelativeSpeeds,
                this::driveRobotRelative,
                new HolonomicPathFollowerConfig(
                        new PIDConstants(SwerveConstants.AutoConstants.TRANSLATION_CONTROLLER_P_COEFF, 0.0, 0.0),
                        new PIDConstants(SwerveConstants.AutoConstants.ROTATION_CONTROLLER_P_COEFF, 0.0, 0.0),
                        SwerveConstants.maxSpeed,
                        0.422930975455813818,
                        new ReplanningConfig()),
                () -> {

                    var alliance = DriverStation.getAlliance();
                    if (alliance.isPresent()) {
                        return alliance.get() == DriverStation.Alliance.Red;
                    }
                    return false;
                },
                this);

        this.visionSubsystem = visionSubsystem;

        boolean isRed = DriverStation.getAlliance().isPresent() && DriverStation.getAlliance().get() == Alliance.Red;

        targetGoalPose = isRed
                ? RED_GOAL_POSE
                : BLUE_GOAL_POSE;
    }

    public SwerveDrive() {
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

        swerveOdometry = new SwerveDrivePoseEstimator(
                SwerveConstants.swerveKinematics,
                getGyroYaw(),
                getModulePositions(),
                new Pose2d());

        AutoBuilder.configureHolonomic(
                this::getPose,
                this::setPose,
                this::getRobotRelativeSpeeds,
                this::driveRobotRelative,
                new HolonomicPathFollowerConfig(
                        new PIDConstants(SwerveConstants.AutoConstants.TRANSLATION_CONTROLLER_P_COEFF, 0.0, 0.0),
                        new PIDConstants(SwerveConstants.AutoConstants.ROTATION_CONTROLLER_P_COEFF, 0.0, 0.0),
                        SwerveConstants.maxSpeed,
                        0.422930975455813818,
                        new ReplanningConfig()),
                () -> {

                    var alliance = DriverStation.getAlliance();
                    if (alliance.isPresent()) {
                        return alliance.get() == DriverStation.Alliance.Red;
                    }
                    return false;
                },
                this);
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

    public void setHeading(Rotation2d heading) {
        swerveOdometry.resetPosition(getGyroYaw(), getModulePositions(),
                new Pose2d(getPose().getTranslation(), heading));
    }

    public void zeroHeading() {
        swerveOdometry.resetPosition(getGyroYaw(), getModulePositions(),
                new Pose2d(getPose().getTranslation(), new Rotation2d()));
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

    public double getDistanceMetersToGoal() {
        return getTargetGoalPose().getDistance(swerveOdometry.getEstimatedPosition().getTranslation());
    }
    
    public double getAngleDegreesToGoal() {
        Translation2d goalPose = getTargetGoalPose();
        Translation2d robotPose = swerveOdometry.getEstimatedPosition().getTranslation();
        Translation2d robotToGoal = goalPose.minus(robotPose);
        return Math.toDegrees(Math.atan2(robotToGoal.getY(), robotToGoal.getX()));
    }

    @Override
    public void periodic() {
        swerveOdometry.update(getGyroYaw(), getModulePositions());

        if (visionSubsystem != null) {
            if (visionSubsystem.getLeftVisionEstimatePresent()) {
                addVisionMeasurement(
                        visionSubsystem.getLeftVisionEstimatePose2d(),
                        visionSubsystem.getLeftVisionEstimateTimestamp());
            }

            if (visionSubsystem.getRightVisionEstimatePresent()) {
                addVisionMeasurement(
                        visionSubsystem.getRightVisionEstimatePose2d(),
                        visionSubsystem.getRightVisionEstimateTimestamp());
            }
        }

        for (SwerveModule mod : mSwerveMods) {
            SmartDashboard.putNumber("Mod " + mod.moduleNumber + " CANcoder", mod.getCANcoder().getDegrees());
            SmartDashboard.putNumber("Mod " + mod.moduleNumber + " Angle", mod.getPosition().angle.getDegrees());
            SmartDashboard.putNumber("Mod " + mod.moduleNumber + " Velocity", mod.getState().speedMetersPerSecond);
        }
    }
}