// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.DigitalOutput;

/** Add your docs here. */
public final class Lights {

    private static final DigitalOutput mLights = new DigitalOutput(Constants.Utility.lightPort);

    public static void setLights(boolean hasNote) {
        if (hasNote) {
            mLights.set(true);
        } else {
            mLights.set(false);
        }
    }

}
