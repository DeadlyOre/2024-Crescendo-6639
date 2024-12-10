// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Intake;

public class IntakeCommand extends Command {

  private final Intake mIntake;
  private final boolean mReverse;
  private final boolean mSensor;

  /** Creates a new IntakeCommand. */
  public IntakeCommand(Intake intake, boolean reverse, boolean sensor) {
    // Use addRequirements() here to declare subsystem dependencies.
    mIntake = intake;
    mReverse = reverse;
    mSensor = sensor;
    addRequirements(mIntake);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    mIntake.eat(mReverse, false, mSensor);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    mIntake.eat(mReverse, true, mSensor);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
