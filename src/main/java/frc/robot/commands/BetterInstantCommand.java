package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.FunctionalCommand;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.Subsystem;

public class BetterInstantCommand extends FunctionalCommand{

    /**
   * Creates a new InstantCommand that runs the given Runnable with the given requirements.
   *
   * @param toRun the Runnable to run
   * @param requirements the subsystems required by this command
   */
  public BetterInstantCommand(Runnable toRun, Subsystem... requirements) {
    super(toRun, () -> {}, interrupted -> {}, () -> false, requirements);
  }

  /**
   * Creates a new InstantCommand with a Runnable that does nothing. Useful only as a no-arg
   * constructor to call implicitly from subclass constructors.
   */
  public BetterInstantCommand() {
    this(() -> {});
  }
    
}
