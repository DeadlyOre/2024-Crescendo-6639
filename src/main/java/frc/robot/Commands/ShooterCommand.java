// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Shooter;

public class ShooterCommand extends Command {
  /** Creates a new ShooterCommand. */
  private final Shooter mShooter;
  private final boolean mReverse;

  public ShooterCommand(Shooter shooter, boolean reverse) {
    // Use addRequirements() here to declare subsystem dependencies.
    mShooter = shooter;
    mReverse = reverse;
    addRequirements(mShooter);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    mShooter.shoot(false, mReverse);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    mShooter.shoot(true, mReverse);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
