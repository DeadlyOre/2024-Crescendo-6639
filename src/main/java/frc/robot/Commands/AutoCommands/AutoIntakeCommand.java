// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Commands.AutoCommands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Sensor;
import frc.robot.subsystems.Intake;

public class AutoIntakeCommand extends Command {

  private final Intake mIntake;

  /** Creates a new AutoIntakeCommand. */
  public AutoIntakeCommand(Intake intake) {
    // Use addRequirements() here to declare subsystem dependencies.
    mIntake = intake;
    addRequirements(mIntake);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if (Sensor.getSensor()) {
      mIntake.eat(false, true, false);
    } else {
      mIntake.eat(false, false, false);
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    mIntake.eat(false, true, false);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
