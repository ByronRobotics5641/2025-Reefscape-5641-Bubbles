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

  public void lift() {
    lead.set(0);
    follow.set(0);
  }
  public void lower() {
    lead.set(-0);
    follow.set(-0);
  }
  public void stopEle() {
    lead.set(0);
    follow.set(0);
  }
  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
