// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Climber;

public class ClimberCommand extends Command {

  private final Climber mClimber;
  private final boolean mStop;
  private final boolean mDown;

  /** Creates a new ClimberCommand. */
  public ClimberCommand(Climber climber, boolean stop, boolean down) {
    // Use addRequirements() here to declare subsystem dependencies.
    mClimber = climber;
    mStop = stop;
    mDown = down;
    addRequirements(mClimber);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    mClimber.climb(mDown, mStop);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    mClimber.climb(mDown, true);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
