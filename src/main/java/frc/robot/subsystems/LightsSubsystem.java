// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.util.Optional;

import com.revrobotics.ColorSensorV3;
import edu.wpi.first.wpilibj.I2C;

import frc.robot.subsystems.CoralSubsystem;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.motorcontrol.Spark;
import edu.wpi.first.wpilibj2.command.SubsystemBase;


public class LightsSubsystem extends SubsystemBase {
  /** Creates a new LightsSubsystem. */
  public LightsSubsystem() {}
  Spark lights = new Spark(0);

  private final I2C.Port i2cPort = I2C.Port.kOnboard;//use I2C port of RIO
  private final ColorSensorV3 colorSensor = new ColorSensorV3(i2cPort);//v3 color sensor

  Optional<Alliance> ally = DriverStation.getAlliance();
  

 public void setAllianceColor() {
  if (ally.isPresent()) {
      System.out.println(ally.get());
      if(ally.get() == Alliance.Red) {
          lights.set (-0.11);
          //System.out.println("Alliance: Red");
      }
      else if (ally.get() == Alliance.Blue)
          lights.set (-0.09);
          //System.out.println("Alliance: Blue");
    }
    else {}
        //System.out.println("No alliance color found");
     }
    
     public void defaultColor(){
      lights.set(-0.03);
     }

     public boolean checkCoral(){
      //use value when nothing is in SF
      if (colorSensor.getProximity() == 2047){
        return true;
      }
      else{
        return false;
      }
    }


  

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    if(checkCoral()) {
      setAllianceColor();
    }
    else {
      defaultColor();
    }
  }
}
