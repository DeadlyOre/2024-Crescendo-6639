// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Climber extends SubsystemBase {

  private final CANSparkMax mLeft;
  private final CANSparkMax mRight;

  /** Creates a new Climber. */
  public Climber() {


    mLeft = new CANSparkMax(Constants.Utility.kLeftClimber, MotorType.kBrushless);
    mRight = new CANSparkMax(Constants.Utility.kRightClimber, MotorType.kBrushless);


    mLeft.restoreFactoryDefaults();
    mRight.restoreFactoryDefaults();

    mRight.setInverted(true);

    mLeft.setIdleMode(Constants.Utility.climberMotorIdleMode);
    mRight.setIdleMode(Constants.Utility.climberMotorIdleMode);

    mLeft.burnFlash();
    mRight.burnFlash();
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  public void climb(boolean down, boolean stop) {
    
    if (stop) {
      mLeft.set(0);
      mRight.set(0);
    } else {
      if (down) {
        mLeft.set(Constants.Utility.climberSpeed);
        mRight.set(Constants.Utility.climberSpeed);
      } else {
        mLeft.set(-Constants.Utility.climberSpeed);
        mRight.set(-Constants.Utility.climberSpeed);
      }
    }
  }
}
