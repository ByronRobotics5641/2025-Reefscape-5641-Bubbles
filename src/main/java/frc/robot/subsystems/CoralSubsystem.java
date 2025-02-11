// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class CoralSubsystem extends SubsystemBase {
  /** Creates a new CoralSubsystem. */
  //two motors one axis, one intake/deposit
  //SuckerFeeder
  public CoralSubsystem() {}
    SparkMax rotate = new SparkMax(3, MotorType.kBrushless);
    SparkMax coral = new SparkMax(4, MotorType.kBrushless);
  

  public void downAngle() {
    rotate.set(0);
  }
  public void upAngle() {
    rotate.set(-0);
  }
  public void stopAngle() {
    rotate.set(0);
  }

  public void coralIn() {
    coral.set(0);
  }
  public void coralOut() {
    coral.set(-0);
  }
  public void coralStop() {
    coral.set(0);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
