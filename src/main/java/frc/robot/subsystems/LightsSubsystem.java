// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.util.Optional;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.motorcontrol.Spark;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class LightsSubsystem extends SubsystemBase {
  /** Creates a new LightsSubsystem. */
  public LightsSubsystem() {}
  Spark lights = new Spark(0);
  Optional<Alliance> ally = DriverStation.getAlliance();
  
 public void setAllianceColor() {
    if (ally.isPresent()) {
      System.out.println(ally.get());
      if(ally.get() == Alliance.Red) {
          lights.set (0.61);
          //System.out.println("Alliance: Red");
      }
      else if (ally.get() == Alliance.Blue)
          lights.set (0.87);
          //System.out.println("Alliance: Blue");
    }
    else 
    {
        //System.out.println("No alliance color found");
    }
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
