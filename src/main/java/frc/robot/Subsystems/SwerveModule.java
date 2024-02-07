// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Subsystems;

import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkPIDController;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.SparkAbsoluteEncoder.Type;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.ModuleConstants;

/** Add your docs here. */
public class SwerveModule {
    private final CANSparkMax mDrivingMotor;
    private final CANSparkMax mTurningMotor;

    private final RelativeEncoder mDrivingEncoder;
    private final AbsoluteEncoder mTurningEncoder;

    private final SparkPIDController mDrivingPIDController;
    private final SparkPIDController mTurningPIDController;

    private double steerOffset;
    private SwerveModuleState mDesiredState = new SwerveModuleState(0.0, new Rotation2d());

    /** 
     * A swerve module with a drive and turning motor
     * PID controllers for both
    */
    public SwerveModule(int driveCANID, int turningCANID, double chassisAngularOffset) {
        mDrivingMotor = new CANSparkMax(driveCANID, MotorType.kBrushless);
        mTurningMotor = new CANSparkMax(turningCANID, MotorType.kBrushless);

        //Set motors to a known state
        mDrivingMotor.restoreFactoryDefaults();
        mTurningMotor.restoreFactoryDefaults();

        //Set PID's and encoders
        mDrivingEncoder = mDrivingMotor.getEncoder();
        mTurningEncoder = mTurningMotor.getAbsoluteEncoder(Type.kDutyCycle);
        mDrivingPIDController = mDrivingMotor.getPIDController();
        mTurningPIDController = mTurningMotor.getPIDController();
        mDrivingPIDController.setFeedbackDevice(mDrivingEncoder);
        mTurningPIDController.setFeedbackDevice(mTurningEncoder);

        //Set Conversion factors
        mDrivingEncoder.setPositionConversionFactor(ModuleConstants.kDriveEncoderPositionFactor);
        mDrivingEncoder.setVelocityConversionFactor(ModuleConstants.kDriveEncoderVelocityFactor);
        mTurningEncoder.setPositionConversionFactor(ModuleConstants.kTurningEncoderPositionFactor);
        mTurningEncoder.setVelocityConversionFactor(ModuleConstants.kTurningEncoderVelocityFactor);
        mTurningEncoder.setInverted(ModuleConstants.kTurningEncoderReversed);

        //Enable steer wraparound
        mTurningPIDController.setPositionPIDWrappingEnabled(true);
        mTurningPIDController.setPositionPIDWrappingMaxInput(ModuleConstants.kTurningMaxOutput);
        mTurningPIDController.setPositionPIDWrappingMinInput(ModuleConstants.kTurningMinOutput);

        //Set PID gains
        mDrivingPIDController.setP(ModuleConstants.kDrivingP);
        mDrivingPIDController.setI(ModuleConstants.kDrivingI);
        mDrivingPIDController.setD(ModuleConstants.kDrivingD);
        mDrivingPIDController.setFF(ModuleConstants.kDrivingFF);
        mDrivingPIDController.setOutputRange(ModuleConstants.kDrivingMinOutput, ModuleConstants.kDrivingMaxOutput);

        mTurningPIDController.setP(ModuleConstants.kTurningP);
        mTurningPIDController.setI(ModuleConstants.kTurningI);
        mTurningPIDController.setD(ModuleConstants.kTurningD);
        mTurningPIDController.setFF(ModuleConstants.kTurningFF);
        mTurningPIDController.setOutputRange(ModuleConstants.kTurningMinOutput, ModuleConstants.kTurningMaxOutput);

        //Final configrations
        mDrivingMotor.setIdleMode(ModuleConstants.kDrivingMotorIdleMode);
        mTurningMotor.setIdleMode(ModuleConstants.kTurningMotorIdleMode);
        mDrivingMotor.setSmartCurrentLimit(ModuleConstants.kDrivingMotorCurrentLimit);
        mTurningMotor.setSmartCurrentLimit(ModuleConstants.kTurningMotorCurrentLimit);

        mDrivingMotor.burnFlash();
        mTurningMotor.burnFlash();

        steerOffset = chassisAngularOffset;
        mDesiredState.angle = new Rotation2d(mTurningEncoder.getPosition());
        mDrivingEncoder.setPosition(0);
    }

    /*
     * Returns the current state of the module
     */
    public SwerveModuleState getState() {
        return new SwerveModuleState(
            mDrivingEncoder.getVelocity(),
            new Rotation2d(mTurningEncoder.getPosition() - steerOffset)
        );
    }

    /*
     * Returns the current position of the module
     */
    public SwerveModulePosition getPosition() {
        return new SwerveModulePosition(
            mDrivingEncoder.getPosition(),
            new Rotation2d(mTurningEncoder.getPosition() - steerOffset)
        );
    }

    /*
     * Sets the desired state for the module
     * 
     * @param desiredState Desired state with speed and angle
     */
    public void setDesiredState(SwerveModuleState desiredState) {
        //Apply angular offset
        SwerveModuleState correctedDesiredState = new SwerveModuleState();
        correctedDesiredState.speedMetersPerSecond = desiredState.speedMetersPerSecond;
        correctedDesiredState.angle = desiredState.angle.plus(Rotation2d.fromRadians(steerOffset));

        // optimize the reference state
        SwerveModuleState optimizedDesiredState = SwerveModuleState.optimize(correctedDesiredState, 
            new Rotation2d(mTurningEncoder.getPosition()));

        //get motors towards desired setpoints
        mDrivingPIDController.setReference(optimizedDesiredState.speedMetersPerSecond, CANSparkMax.ControlType.kVelocity);

        if (Math.abs(optimizedDesiredState.speedMetersPerSecond) > DriveConstants.kMinSpeedMetersPerSecond) {
            mTurningPIDController.setReference(optimizedDesiredState.angle.getRadians(), CANSparkMax.ControlType.kPosition);
        }

        mDesiredState = desiredState;
    }

    public void stop() {
        mDrivingMotor.set(0.0);
        mTurningMotor.set(0.0);
    }

    public void resetEncoders() {
        mDrivingEncoder.setPosition(0);
    }
}
