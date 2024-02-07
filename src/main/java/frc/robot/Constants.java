// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;



import com.revrobotics.CANSparkBase.IdleMode;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.util.Units;

//Holds the constants for the robot
public class Constants {

    public static final class DriveConstants {
        //Drive Motor CAN ID's
        public static final int frontLeftDrive = 0; //TODO Set all can ID's Correctly
        public static final int frontRightDrive = 0;
        public static final int backLeftDrive = 0;
        public static final int backRightDrive = 0;

        public static final int frontLeftTurn = 0;
        public static final int frontRightTurn = 0;
        public static final int backLeftTurn = 0;
        public static final int backRightTurn = 0;

        //Chassis Configuration
        //Distance between left and right wheels
        public static final double kTrackWidth = Units.inchesToMeters(0); //TODO Set correct values here
        //Distance between front and back wheels
        public static final double kWheelBase = Units.inchesToMeters(0);

        public static final SwerveDriveKinematics kDriveKinematics = new SwerveDriveKinematics( 
            new Translation2d(kWheelBase / 2, kTrackWidth / 2), //Kinematics for swerve, +X is forward, +Y is left
            new Translation2d(kWheelBase / 2, -kTrackWidth / 2),
            new Translation2d(-kWheelBase / 2, kTrackWidth / 2),
            new Translation2d(- kWheelBase / 2, -kTrackWidth / 2)
        );

        //Angular offsets of swerve modules relative to the chassis in radians
        public static final double kFrontLeftAngularOffset = -Math.PI / 2;
        public static final double kFrontRightAngularOffset = 0;
        public static final double kBackLeftAngularOffset = Math.PI;
        public static final double kBackRightAngularOffset = Math.PI / 2;

        public static final double kMaxSpeedMetersPerSecond = 4.0; //Can be changed whenever
        public static final double kMinSpeedMetersPerSecond = 0.01;
        public static final double kMaxAngularSpeed = 2 * Math.PI; //radians per second

        public static final boolean kGyroReversed = true;
        public static final int kGyroDirection = kGyroReversed ? -1 : 1;
    }

    public static final class ModuleConstants {

        public static final int kNeoFreeSpeedRPM = 5676;

        public static final int kDriveMotorPinionTeeth = 13; //TODO: Verify this is true

        //set to true bc of how Maxswerve is built
        public static final boolean kTurningEncoderReversed = true; 

        //Calculations for drive motor conversion and feed foreward
        public static final double kDriveMotorFreeSpeedRPS = kNeoFreeSpeedRPM / 60;
        public static final double kWheelDiameterMeters = 0.0762; //TODO: verify this is correct
        public static final double kWheelCircumferenceMeters = kWheelDiameterMeters * Math.PI;
        public static final double kDrivingMotorReduction = (45.0 * 22) / (kDriveMotorPinionTeeth * 15);
        public static final double kDriveWheelFreeSpeedRPS = (kDriveMotorFreeSpeedRPS * kWheelCircumferenceMeters) / kDrivingMotorReduction;
        
        public static final double kDriveEncoderPositionFactor = kWheelCircumferenceMeters / kDrivingMotorReduction; //Meters
        public static final double kDriveEncoderVelocityFactor = kWheelCircumferenceMeters / kDrivingMotorReduction / 60.0; //Meters Per Second

        public static final double kTurningEncoderPositionFactor = (2 * Math.PI); //radians
        public static final double kTurningEncoderVelocityFactor = (2 * Math.PI) / 60.0; //radians per second

        //Drivetrain PID factors
        public static final double kDrivingP = 0.04;
        public static final double kDrivingI = 0;
        public static final double kDrivingD = 0;
        public static final double kDrivingFF = 1 / kDriveWheelFreeSpeedRPS;
        public static final double kDrivingMinOutput = -1;
        public static final double kDrivingMaxOutput = 1;

        public static final double kTurningP = 1;
        public static final double kTurningI = 0;
        public static final double kTurningD = 0;
        public static final double kTurningFF = 0;
        public static final double kTurningMinOutput = -1;
        public static final double kTurningMaxOutput = 1;

        public static final IdleMode kDrivingMotorIdleMode = IdleMode.kBrake;
        public static final IdleMode kTurningMotorIdleMode = IdleMode.kBrake;

        public static final int kDrivingMotorCurrentLimit = 50; //both in amps
        public static final int kTurningMotorCurrentLimit = 20;
    }

    public static final class IOConstants {
        public static final int kDriveController = 0;
        public static final double controllerDeadband = 0.2;
    }
}
