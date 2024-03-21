package frc.robot.subsystems.swerve;

import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModulePosition;

import java.nio.file.Path;
import java.sql.Driver;
import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;

import com.ctre.phoenix6.configs.Pigeon2Configuration;
import com.ctre.phoenix6.hardware.Pigeon2;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.path.PathPlannerPath;
import com.pathplanner.lib.path.PathPlannerTrajectory;
import com.pathplanner.lib.util.GeometryUtil;
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
import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.vision.LimelightHelpers;

public class SwerveDrive extends SubsystemBase {
    public SwerveDrivePoseEstimator swerveOdometry;
    public SwerveModule[] mSwerveMods;
    public Pigeon2 gyro;

    private final Pose2d RED_GOAL_POSE = new Pose2d(new Translation2d(16.58, 5.55), Rotation2d.fromDegrees(180));
    private final Pose2d BLUE_GOAL_POSE = new Pose2d(new Translation2d(-0.04, 5.55), Rotation2d.fromDegrees(0));

    private final Translation2d RED_GOAL_POSE_ROTATION_TARGET = new Translation2d(16.579342 - Units.inchesToMeters(12),
            5.547867999999999);
    private final Translation2d BLUE_GOAL_POSE_ROTATION_TARGET = new Translation2d(
            -0.038099999999999995 + Units.inchesToMeters(12), 5.547867999999999);

    private Pose2d targetGoalPose;
    private Translation2d targetGoalPoseRotationTarget;

    Field2d field2d;

    // Hardcode
    boolean isRed = false;

    private GenericEntry shooterOffset;

    public SwerveDrive() {

        shooterOffset = Shuffleboard.getTab("SmartDashboard").add("Shooter offset", 0).getEntry();

        field2d = new Field2d();
        SmartDashboard.putData(field2d);

        gyro = new Pigeon2(SwerveConstants.pigeonID);
        gyro.getConfigurator().apply(new Pigeon2Configuration());
        gyro.setYaw(0);

        mSwerveMods = new SwerveModule[] {
                new SwerveModule(0, SwerveConstants.Mod0.constants),
                new SwerveModule(1, SwerveConstants.Mod1.constants),
                new SwerveModule(2, SwerveConstants.Mod2.constants),
                new SwerveModule(3, SwerveConstants.Mod3.constants)
        };
        resetModulesToAbsolute();

        var stateStdDevs = VecBuilder.fill(0.1, 0.1, 0.1);
        var visionStdDevs = VecBuilder.fill(0.7, 0.7, 999999999);

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
                isRed(),
                this);

        setTargetGoal();

        //setCurrentAlliance();
    }

    // public void setCurrentAlliance() {

    //     if(DriverStation.getAlliance().isPresent())
    //     {
    //         isRed = DriverStation.getAlliance().get() == Alliance.Red;
            
    //     }

    //     else {
    //         isRed = false;
    //     }
                

    //     setTargetGoal();
    // }

    public BooleanSupplier isRed() {
        return () -> isRed;
    }

    public void setTargetGoal() {
        //isRed = DriverStation.getAlliance().get() == Alliance.Red;
        targetGoalPose = isRed
                ? RED_GOAL_POSE
                : BLUE_GOAL_POSE;

        targetGoalPoseRotationTarget = isRed
                ? RED_GOAL_POSE_ROTATION_TARGET
                : BLUE_GOAL_POSE_ROTATION_TARGET;

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

    public double getHeadingDegrees() {
        double heading = getHeading().getDegrees();
        if (heading < 0) {
            heading += 360;
        }
        return heading;
    }

    public void setHeading(Rotation2d heading) {
        swerveOdometry.resetPosition(getGyroYaw(), getModulePositions(),
                new Pose2d(getPose().getTranslation(), heading));

    }

    public void zeroHeading() {
        if (isRed().getAsBoolean()) {
            new Rotation2d();
            swerveOdometry.resetPosition(getGyroYaw(), getModulePositions(),
                    new Pose2d(getPose().getTranslation(), Rotation2d.fromDegrees(180)));
        }

        else {
            swerveOdometry.resetPosition(getGyroYaw(), getModulePositions(),
                    new Pose2d(getPose().getTranslation(), new Rotation2d()));
        }
    }

    public Rotation2d getGyroYaw() {
        return Rotation2d.fromDegrees(gyro.getYaw().getValue());
    }

    public void resetModulesToAbsolute() {
        for (SwerveModule mod : mSwerveMods) {
            mod.resetToAbsolute();
        }
    }

    public void setInitialPose(String pathName) {

        PathPlannerPath path = PathPlannerPath.fromPathFile(pathName);
        if (isRed().getAsBoolean()) {
            setPose(GeometryUtil.flipFieldPose(path.getPreviewStartingHolonomicPose()));
        } else {
            setPose(path.getPreviewStartingHolonomicPose());
        }
    }

    public void setInitialPose(PathPlannerPath startPath) {

        PathPlannerPath path = startPath;
        if (isRed().getAsBoolean()) {
            setPose(GeometryUtil.flipFieldPose(path.getPreviewStartingHolonomicPose()));
        } else {
            setPose(path.getPreviewStartingHolonomicPose());
        }
    }

    public Command followPathCommand(String pathName, boolean initialPath) {
        PathPlannerPath path = PathPlannerPath.fromPathFile(pathName);

        return AutoBuilder.followPath(path);
    }

    public Command followPathCommand(PathPlannerPath path, boolean initialPath) {
        return AutoBuilder.followPath(path);
    }

    private Pose2d getTargetGoalPose() {
        return targetGoalPose;
    }

    private Translation2d getTargetGoalPoseRotationTarget() {
        return targetGoalPoseRotationTarget;
    }

    // returns the distance, in meters, from the center of the robot to the target
    // goal
    public double getDistanceMetersToGoal() {
        return getPose().getTranslation().getDistance(getTargetGoalPose().getTranslation());
    }

    public double getDistanceMetersToGoal(Translation2d pose) {
        return pose.getDistance(BLUE_GOAL_POSE.getTranslation());
    }

    // returns the angle, in degrees, that the robot needs to be pointing in order
    // to be pointing at the target goal
    public double getAngleDegreesToGoal() {
        Translation2d goalPose = getTargetGoalPoseRotationTarget();
        Translation2d robotPose = getPose().getTranslation();
        Translation2d robotToGoal = goalPose.minus(robotPose);
        double target = Math.toDegrees(Math.atan2(robotToGoal.getY(), robotToGoal.getX())) + 180;
        if (target >= 5 && target <= 40) {
            target -= 5;
            return target;
        } else if (target >= 325 && target <= 350) {
            target += 5;
            return target;
        }
        return target;
    }

    public DoubleSupplier getDistanceOffsetSupplier() {
        return () -> shooterOffset.getDouble(0);
    }

    public double getTx() {
        return LimelightHelpers.getTX("limelight-driver");
    }

    public boolean seesNote()
    {
        return LimelightHelpers.getTV("limelight-driver");
    }

    @Override
    public void periodic() {

        LimelightHelpers.PoseEstimate limelightMeasurement = LimelightHelpers
                .getBotPoseEstimate_wpiBlue("limelight-main");
        

        if (limelightMeasurement.tagCount >= 1 && DriverStation.isTeleopEnabled() && limelightMeasurement.rawFiducials[0].ambiguity < 0.9) {// if (limelightMeasurement.tagCount >= 2) {
            // swerveOdometry.setVisionMeasurementStdDevs(VecBuilder.fill(.7, .7, 9999999));
            swerveOdometry.addVisionMeasurement(
                    limelightMeasurement.pose,
                    limelightMeasurement.timestampSeconds);
        }

        
        swerveOdometry.update(getGyroYaw(), getModulePositions());

        field2d.setRobotPose(getPose());
        // SmartDashboard.putString("Allian", DriverStation.getAlliance().toString());
        SmartDashboard.putNumber("distance", getDistanceMetersToGoal());
        SmartDashboard.putString("Pose", getPose().toString());
        SmartDashboard.putNumber("heading", getHeadingDegrees());
        SmartDashboard.putNumber("target", getAngleDegreesToGoal());
    }
}