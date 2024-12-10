// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkLowLevel.MotorType;

import frc.robot.Constants;

public class Shooter extends SubsystemBase {
  /** Creates a new Shooter. */
  private final CANSparkMax m_Rightmotor;
  private final CANSparkMax m_Leftmotor;

  public Shooter() {
    m_Rightmotor = new CANSparkMax(Constants.Utility.kRightShooter, MotorType.kBrushless);
    m_Leftmotor = new CANSparkMax(Constants.Utility.kLeftShooter, MotorType.kBrushless);

    m_Rightmotor.restoreFactoryDefaults();
    m_Leftmotor.restoreFactoryDefaults();

    m_Rightmotor.setIdleMode(Constants.Utility.shootingMotorIdleMode);
    m_Leftmotor.setIdleMode(Constants.Utility.shootingMotorIdleMode);

    m_Leftmotor.burnFlash();
    m_Rightmotor.burnFlash();
  }

  //Turn off and on Shooter
  public void shoot(boolean stop, boolean reverse) {
    if (stop) {
      m_Leftmotor.set(0);
      m_Rightmotor.set(0);
    } else {
      if (reverse) {
        m_Leftmotor.set(-Constants.Utility.shooterSpeed * 0.1);
        m_Rightmotor.set(Constants.Utility.shooterSpeed * 0.1);
      } else {
        m_Leftmotor.set(Constants.Utility.shooterSpeed);
        m_Rightmotor.set(-Constants.Utility.shooterSpeed);
      }
    }
  }


  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
