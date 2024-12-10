// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.cscore.UsbCamera;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.XboxController;
//import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Commands.ClimberCommand;
import frc.robot.Commands.IntakeCommand;
//import frc.robot.Commands.LightCommand;
import frc.robot.Commands.ShooterCommand;
import frc.robot.Commands.AutoCommands.AutoIntakeCommand;
import frc.robot.Constants.OIConstants;
import frc.robot.subsystems.Climber;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.Intake;
//import frc.robot.subsystems.Lights;
import frc.robot.subsystems.Shooter;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
//import com.pathplanner.lib.util.PathPlannerLogging;

/*
 * This class is where the bulk of the robot should be declared.  Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls).  Instead, the structure of the robot
 * (including subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {

  
  XboxController m_driverController = new XboxController(OIConstants.kDriverControllerPort);
  XboxController m_copilotController = new XboxController(OIConstants.kCopilotControllerPort);

  // The robot's subsystems
  private final DriveSubsystem m_robotDrive = new DriveSubsystem(m_driverController);
  private final Shooter mShooter = new Shooter();
  private final Intake mIntake = new Intake();
  //private IntakeState mIntakeState = IntakeState.IDLE;
 // private final Lights mLights = new Lights();
  private final Climber mClimber = new Climber();

  private final UsbCamera IntakeCamera;
  //private final UsbCamera ShooterCamera; 
  private final NetworkTableEntry cameraSelection;

  // The driver's controller

  private final SendableChooser<Command> autoChooser;
  //private final Field2d field;

  /**
   * The container for the robot. Contains subsystems, OI devices, and commands.
   */
  public RobotContainer() {
    // Configure the button bindings

    // NamedCommands.registerCommand("AShootCommand", new ShooterCommand(mShooter, false)
    //   .withTimeout(1).andThen(new ShooterCommand(mShooter, false)
    //   .alongWith(new IntakeCommand(mIntake, false))).withTimeout(0.5));

    Command shoot1 = new WaitCommand(0.14)
      .andThen(new IntakeCommand(mIntake, false, false).withTimeout(0.5));
    
    Command shoot2 = new ShooterCommand(mShooter, false).withTimeout(0.69);

    NamedCommands.registerCommand("AShootCommand", shoot1.alongWith(shoot2));

    NamedCommands.registerCommand("AIntakeCommand", new AutoIntakeCommand(mIntake));
    
    //NamedCommands.registerCommand("AReadyShotCommand", new IntakeCommand(mIntake, true) __Hopefully wont need this again
    //  .alongWith(new ShooterCommand(mShooter, true)).withTimeout(0.15));

    configureButtonBindings();
    
    IntakeCamera = CameraServer.startAutomaticCapture(0);
    IntakeCamera.setResolution(640,480);
    IntakeCamera.setFPS(30);

    //ShooterCamera = CameraServer.startAutomaticCapture(1);
    //ShooterCamera.setResolution(640,480);
    //ShooterCamera.setFPS(30);


    cameraSelection = NetworkTableInstance.getDefault().getTable("").getEntry("Camera Selection");

    autoChooser = AutoBuilder.buildAutoChooser();
    // field = new Field2d();
    // SmartDashboard.putData("Field", field);

    // PathPlannerLogging.setLogCurrentPoseCallback((pose) -> {
    //   field.setRobotPose(pose); // Can modify pose here
    // });

    // PathPlannerLogging.setLogTargetPoseCallback((pose) -> {
    //   field.getObject("target pose").setPose(pose); // Can modify pose here
    // });

    // PathPlannerLogging.setLogActivePathCallback((poses) -> {
    //   field.getObject("path").setPoses(poses); // Can modify pose here
    // });

    // Configure default commands
    //mLights.setDefaultCommand(new LightCommand(mLights));
    //mClimber.setDefaultCommand(new ClimbOneCommand(mClimber, m_driverController));
    m_robotDrive.setDefaultCommand(
        // The left stick controls translation of the robot.
        // Turning is controlled by the X axis of the right stick.
        new RunCommand(
            () -> m_robotDrive.drive(
                -MathUtil.applyDeadband(m_driverController.getLeftY(), OIConstants.kDriveDeadband),
                -MathUtil.applyDeadband(m_driverController.getLeftX(), OIConstants.kDriveDeadband),
                -MathUtil.applyDeadband(m_driverController.getRightX(), OIConstants.kDriveDeadband),
                true, true),
            m_robotDrive));
    
    SmartDashboard.putData("Auto Chooser", autoChooser);
  }

  /*
   * Use this method to define your button->command mappings. Buttons can be
   * created by
   * instantiating a {@link edu.wpi.first.wpilibj.GenericHID} or one of its
   * subclasses ({@link
   * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then calling
   * passing it to a
   * {@link JoystickButton}.
   */
  private void configureButtonBindings() {
    // new JoystickButton(m_driverController, XboxController.Button.kRightStick.value)
    //     .whileTrue(new RunCommand(
    //         () -> m_robotDrive.setX(),
    //         m_robotDrive));
    
    new JoystickButton(m_driverController, XboxController.Button.kStart.value)
        .whileTrue(new RunCommand(
            () -> m_robotDrive.zeroHeading(), m_robotDrive));
    
    new JoystickButton(m_driverController, XboxController.Button.kA.value)
        .whileTrue(new ShooterCommand(mShooter, false));
    
    new JoystickButton(m_driverController, XboxController.Button.kB.value)
        .whileTrue(new ShooterCommand(mShooter, true)); //run the shooter in reverse
    
    new JoystickButton(m_driverController, XboxController.Button.kRightBumper.value)
        .whileTrue(new IntakeCommand(mIntake, false, false));

    new JoystickButton(m_driverController, XboxController.Button.kLeftBumper.value)
        .whileTrue(new IntakeCommand(mIntake, true, false));
      
    new JoystickButton(m_driverController, XboxController.Button.kX.value)
        .whileTrue(new ClimberCommand(mClimber, false, true));
      
    new JoystickButton(m_driverController, XboxController.Button.kY.value)
        .whileTrue(new ClimberCommand(mClimber, false, false));

    //COPILOT BUTTONS///////////////////////////////////

    new JoystickButton(m_copilotController, XboxController.Button.kRightBumper.value)
        .whileTrue(new IntakeCommand(mIntake, false, false));

     new JoystickButton(m_copilotController, XboxController.Button.kA.value)
        .whileTrue(new ShooterCommand(mShooter, false));
    
    new JoystickButton(m_copilotController, XboxController.Button.kB.value)
        .whileTrue(new ShooterCommand(mShooter, true)); //run the shooter in reverse

    // new JoystickButton(m_copilotController, XboxController.Button.kX.value)
    //     .whileTrue(new ClimberCommand(mClimber, false, true));


    new JoystickButton(m_copilotController, XboxController.Button.kLeftBumper.value)
        .whileTrue(new IntakeCommand(mIntake, true, false));
      
    new JoystickButton(m_copilotController, XboxController.Button.kY.value)
        .whileTrue(new ClimberCommand(mClimber, false, false));
    
    new JoystickButton(m_copilotController, XboxController.Button.kX.value)
        .whileTrue(new IntakeCommand(mIntake, false, true));
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    //return new PathPlannerAuto("Spin");
    return autoChooser.getSelected();
  }
}
