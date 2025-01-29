// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.


/***********************************************
* More information available at:               *
* https://docs.revrobotics.com/revlib/24-to-25 *
************************************************/
package frc.robot.subsystems;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.ClosedLoopSlot;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.ClosedLoopConfig.FeedbackSensor;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class ExampleRevSubsystem extends SubsystemBase {
  /** Creates a new ExampleRevSubsystem. */

  SparkMax motor = new SparkMax(1, MotorType.kBrushless);
  SparkClosedLoopController motorPID = motor.getClosedLoopController();
  SparkMaxConfig config =  new SparkMaxConfig();
  RelativeEncoder motorEncoder = motor.getEncoder();

  //Configures a new object
  public ExampleRevSubsystem() {
    //configure motors
    config
    .inverted(false)
    .idleMode(IdleMode.kBrake); //what happens when speed = 0

    config.encoder
    .positionConversionFactor(1000)
    .velocityConversionFactor(1000);

    config.closedLoop
    .feedbackSensor(FeedbackSensor.kPrimaryEncoder)//use neo encoder
    .pid(1.0,0,0); //adjust as needed

      // Initialize dashboard values
    SmartDashboard.setDefaultNumber("Target Position", 0);
    SmartDashboard.setDefaultNumber("Target Velocity", 0);
    SmartDashboard.setDefaultBoolean("Control Mode", false);
    SmartDashboard.setDefaultBoolean("Reset Encoder", false);
    
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run

    //this is where PID position targets should be implemented. currently set by DS.
    double targetPosition = SmartDashboard.getNumber("Target Position", 0);
    motorPID.setReference(targetPosition, ControlType.kPosition, ClosedLoopSlot.kSlot0);
  }
}
