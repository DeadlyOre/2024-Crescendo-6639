// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Sensor;

public class Intake extends SubsystemBase {

  private final CANSparkMax mLeft;
  private final CANSparkMax mRight;
  /** Creates a new Intake. */
  public Intake() {
    mLeft = new CANSparkMax(Constants.Utility.kLeftIntake, MotorType.kBrushless);
    mRight = new CANSparkMax(Constants.Utility.kRightIntake, MotorType.kBrushless);

    mLeft.restoreFactoryDefaults();
    mRight.restoreFactoryDefaults();

    mLeft.setIdleMode(Constants.Utility.intakeMotorIdleMode);
    mRight.setIdleMode(Constants.Utility.intakeMotorIdleMode);

    mLeft.burnFlash();
    mRight.burnFlash();
  }

  public void eat(boolean reverse, boolean stop, boolean sensor) {
    if (stop || (Sensor.getSensor() && sensor)) {
      mLeft.set(0);
      mRight.set(0);
    } else {
      if (reverse) { //controller 1, sensor has no effect
        mLeft.set(-Constants.Utility.intakeSpeed);
        mRight.set(-Constants.Utility.intakeSpeed);
      } else {
        mLeft.set(Constants.Utility.intakeSpeed);
        mRight.set(Constants.Utility.intakeSpeed);
      }
    }
  }
  

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
