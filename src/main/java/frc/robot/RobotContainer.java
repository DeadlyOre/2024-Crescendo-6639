// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.RunCommand;
import frc.robot.Constants.IOConstants;
import frc.robot.Subsystems.DriveSubsystem;

public class RobotContainer {

  private final DriveSubsystem mRobotDrive = new DriveSubsystem();

  XboxController mDriveController = new XboxController(IOConstants.kDriveController);

  public RobotContainer() {
    configureBindings();

    mRobotDrive.setDefaultCommand(

      new RunCommand(
        () -> mRobotDrive.drive(
          -MathUtil.applyDeadband(mDriveController.getLeftY(), IOConstants.controllerDeadband),
          -MathUtil.applyDeadband(mDriveController.getLeftX(), IOConstants.controllerDeadband),
          -MathUtil.applyDeadband(mDriveController.getRightX(), IOConstants.controllerDeadband)),
        mRobotDrive)
    );
  }

  private void configureBindings() {}

  public Command getAutonomousCommand() {
    return Commands.print("No autonomous command configured");
  }
}
