// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.SwerveSubsystem;

import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;

public class SwerveDefault extends Command {

    private final SwerveSubsystem swerve;
    private final DoubleSupplier vX;
    private final DoubleSupplier vY;
    private final DoubleSupplier omega;

    public SwerveDefault(SwerveSubsystem swerve, DoubleSupplier vX, DoubleSupplier vY, DoubleSupplier omega,
            BooleanSupplier driveMode) {
        this.swerve = swerve;
        this.vX = vX;
        this.vY = vY;
        this.omega = omega;
        swerve.getSwerveController();

        addRequirements(swerve);
    }

    @Override
    public void initialize() {
    }

    @Override
    public void execute() {
        double xVelocity = Math.pow(vX.getAsDouble(), 1);
        double yVelocity = Math.pow(vY.getAsDouble(), 1);
        double angVelocity = Math.pow(omega.getAsDouble(), 1);
        
        SmartDashboard.putNumber("vX", xVelocity);
        SmartDashboard.putNumber("vY", yVelocity);
        SmartDashboard.putNumber("omega", angVelocity);

        swerve.driveFieldOriented(new ChassisSpeeds(
            xVelocity * swerve.maximumSpeed,
            yVelocity  * swerve.maximumSpeed,
            angVelocity * swerve.getSwerveController().config.maxAngularVelocity
        ));
    }

    @Override
    public void end(boolean interrupted) {
    }

    @Override
    public boolean isFinished() {
        return false;
    }
}