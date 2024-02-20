// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.robot.commands.defaultCommands.IntakeDefaultCommand;
import frc.robot.commands.defaultCommands.SwerveDriveTeleop;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.shooter.ShooterSubsystem;
import frc.robot.subsystems.swerve.SwerveDrive;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;

public class RobotContainer {

  private final SwerveDrive swerveSubsystem = new SwerveDrive();

  private final CommandXboxController driverController = new CommandXboxController(0);
  private final ShooterSubsystem shooter = new ShooterSubsystem();

  IntakeSubsystem intake = new IntakeSubsystem();

  public RobotContainer() {

    configureBindings();
    swerveSubsystem.setDefaultCommand(new SwerveDriveTeleop(
        swerveSubsystem,
        () -> -adjustInput(driverController.getLeftY()),
        () -> -adjustInput(driverController.getLeftX()),
        () -> adjustInput(driverController.getRightX()),
        () -> false));
      intake.setDefaultCommand(new IntakeDefaultCommand(intake, () -> (driverController.getRightTriggerAxis() - driverController.getLeftTriggerAxis())));

      new Trigger(driverController.a()).onTrue(new InstantCommand(() -> shooter.setMax()));
      new Trigger(driverController.b()).onTrue(new InstantCommand(() -> shooter.stopShooterMotors()));
      

  }

  

  private double adjustInput(double input) {
    return Math.copySign(Math.pow(Math.abs(input), 1.75), input);
  }

  private void configureBindings() {
    driverController.povDown().onTrue(new InstantCommand(() -> swerveSubsystem.zeroHeading()));
  }

  public Command getAutonomousCommand() {
    return swerveSubsystem.followPathCommand("testPath", true);
  }
}
