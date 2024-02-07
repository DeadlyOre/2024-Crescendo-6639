// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Subsystems;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.DriveConstants;
import com.kauailabs.navx.frc.AHRS;

public class DriveSubsystem extends SubsystemBase {

  //Create Swerve Modules
  private final SwerveModule mFrontLeft = new SwerveModule(
    DriveConstants.frontLeftDrive, 
    DriveConstants.frontLeftTurn, 
    DriveConstants.kFrontLeftAngularOffset);
  
  private final SwerveModule mFrontRight = new SwerveModule(
    DriveConstants.frontRightDrive, 
    DriveConstants.frontRightTurn, 
    DriveConstants.kFrontRightAngularOffset);
  
  private final SwerveModule mBackLeft = new SwerveModule(
    DriveConstants.backLeftDrive, 
    DriveConstants.backLeftTurn, 
    DriveConstants.kBackLeftAngularOffset);
  
  private final SwerveModule mBackRight = new SwerveModule(
    DriveConstants.backRightDrive, 
    DriveConstants.backRightTurn, 
    DriveConstants.kBackRightAngularOffset);
  
  //The gyro
  private final AHRS mGyro = new AHRS();
  private final int mGDir = DriveConstants.kGyroDirection;

  //odometry class for traching robot pose
  SwerveDriveOdometry mOdometry = new SwerveDriveOdometry (
    DriveConstants.kDriveKinematics,
    Rotation2d.fromDegrees(mGDir * (mGyro.getAngle())),
    new SwerveModulePosition[] {
      mFrontLeft.getPosition(),
      mFrontRight.getPosition(),
      mBackLeft.getPosition(),
      mBackRight.getPosition()
    });
  
  private boolean mRobotRelative = true;
  private Translation2d rotationCenter = new Translation2d();


  /** Creates a new DriveSubsystem. */
  public DriveSubsystem() {}

  @Override
  public void periodic() {
    // Updates robot odometry every period
    mOdometry.update(
      Rotation2d.fromDegrees(mGDir * (mGyro.getAngle())),
      new SwerveModulePosition[] {
        mFrontLeft.getPosition(),
        mFrontRight.getPosition(),
        mBackLeft.getPosition(),
        mBackRight.getPosition()
      });
  }

  /*
   * Returns the estimated pose of the swerve bot
   */
  public Pose2d getPose() {
    return mOdometry.getPoseMeters();
  }

  /*
   * Resets odometry to the specified pose
   */
  public void resetOdometry(Pose2d pose) {
    mOdometry.resetPosition(
      Rotation2d.fromDegrees(mGDir * (mGyro.getAngle())), 
      new SwerveModulePosition[] {
        mFrontLeft.getPosition(),
        mFrontRight.getPosition(),
        mBackLeft.getPosition(),
        mBackRight.getPosition()
      }, 
      pose);
  }

  /*
   * Drives the robot relative to the robot
   */
  public void drive(double xSpeed, double ySpeed, double rot) {
    double xVelMetersPerSecond = xSpeed * DriveConstants.kMaxSpeedMetersPerSecond;
    double yVelMetersPerSecond = ySpeed * DriveConstants.kMaxSpeedMetersPerSecond;
    double rotationRadiansPerSecond = rot * DriveConstants.kMaxAngularSpeed;

    ChassisSpeeds chassisSpeeds = mRobotRelative 
    ? new ChassisSpeeds(xVelMetersPerSecond, yVelMetersPerSecond, rotationRadiansPerSecond)
    : ChassisSpeeds.fromFieldRelativeSpeeds(xVelMetersPerSecond, yVelMetersPerSecond, rotationRadiansPerSecond, mGyro.getRotation2d());

    SwerveModuleState[] moduleStates = DriveConstants.kDriveKinematics.toSwerveModuleStates(chassisSpeeds, rotationCenter);

    SwerveDriveKinematics.desaturateWheelSpeeds(moduleStates, DriveConstants.kMaxSpeedMetersPerSecond);

    mFrontLeft.setDesiredState(moduleStates[0]);
    mFrontRight.setDesiredState(moduleStates[1]);
    mBackLeft.setDesiredState(moduleStates[2]);
    mBackRight.setDesiredState(moduleStates[3]);
  }

  public void setX() {
    mFrontLeft.setDesiredState(new SwerveModuleState(0, Rotation2d.fromDegrees(45)));
    mFrontRight.setDesiredState(new SwerveModuleState(0, Rotation2d.fromDegrees(-45)));
    mBackLeft.setDesiredState(new SwerveModuleState(0, Rotation2d.fromDegrees(-45)));
    mBackRight.setDesiredState(new SwerveModuleState(0, Rotation2d.fromDegrees(45)));
  }

  public void stop() {
    mFrontLeft.stop();
    mFrontRight.stop();
    mBackLeft.stop();
    mBackRight.stop();
  }

  public void setModuleStates(SwerveModuleState[] desiredStates) {
    SwerveDriveKinematics.desaturateWheelSpeeds(desiredStates, DriveConstants.kMaxSpeedMetersPerSecond);
    mFrontLeft.setDesiredState(desiredStates[0]);
    mFrontRight.setDesiredState(desiredStates[1]);
    mBackLeft.setDesiredState(desiredStates[2]);
    mBackRight.setDesiredState(desiredStates[3]);
  }

  public void resetEncoders() {
    mFrontLeft.resetEncoders();
    mFrontRight.resetEncoders();
    mBackLeft.resetEncoders();
    mBackRight.resetEncoders();
  }

  public void zeroHeading() {
    mGyro.reset();
  }

  public double getHeading() {
    return mGyro.getRotation2d().getDegrees();
  }

  public void setRobotRelative() {
    this.mRobotRelative = true;
  }

  public void setFieldRelative() {
    this.mRobotRelative = false;
  }

  public boolean getRobotRelative() {
    return mRobotRelative;
  }
}
