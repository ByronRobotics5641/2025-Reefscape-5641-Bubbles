// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class ElevatorSubsystem extends SubsystemBase {
  /** Creates a new ElevatorSubsystem. */
  //Attatches to SuckerFish/CoralMachine
  // D-pad/pov up = 0°, bottom = 180°
  //two motors one lead, one follow 

  public ElevatorSubsystem() {}
  SparkMax lead = new SparkMax(5, MotorType.kBrushless);
  SparkMax follow = new SparkMax(6, MotorType.kBrushless);

  // stick value is DoubleSupplier, passed in through command or instant command with ()->m_manipController.getLeftY()
  public void eleAngle(double leftStickValue, boolean nothing) {
    if(nothing) {
      if(leftStickValue == 0 && nothing) {
        lead.set(leftStickValue * 0);
        follow.set(leftStickValue * 0);
      }
      else{
        lead.set(leftStickValue * 0);
        follow.set(leftStickValue * 0);
      }
    }
  }
  /*public void eleDriver(double speed) {
    lead.set(speed);
    follow.set(speed);
  }
  public void eleLift() {
    lead.set(0);
    follow.set(0);
  }
  public void eleDown() {
    lead.set(0);
    follow.set(0);
  }
  public void eleStop() {
    lead.set(0);
    lead.set(0);
  }*/


  
  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
