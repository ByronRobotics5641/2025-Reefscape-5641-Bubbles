// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.


/***************************************************************
* This class provides a generic example of a PID configuration *
* using Spark Max, but can use any motor with minor changes.   *
****************************************************************/

package frc.robot.subsystems;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;


import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class ExamplePID extends SubsystemBase {
  
 //used for hank to drive ten feet 
 //will not work for sweve drive unless corrected

  private final double kDriveTick2Feet = 1.0 / 21 * 6 * Math.PI / 12;

  final double kP = 0.4;
  final double kI = 0.0;
  final double iLimit = 1;
  final double kD = 0.08;

  double setpoint = 0;
  double errorSum = 0;
  double lastTimestamp = 0;
  double lastError = 0;

  SparkMax motor = new SparkMax(0, MotorType.kBrushless);
  RelativeEncoder encoder;

  //Configures a new object
  public ExamplePID() {
    encoder = motor.getEncoder();
    encoder.setPosition(0);
    errorSum = 0;
    lastError = 0;
    lastTimestamp = Timer.getFPGATimestamp();

  }


  /************************************************
   * This method sets the distance to be traveled.
   * @param setpoint
   ************************************************/
  public void setSetpoint(double setpoint){
    this.setpoint = setpoint;
    //encoder.setPosition(0); resets position... not useful for our subsystems as described
  }

  /******************************************************************
   * This method calculates the distance traveled based on encoder
   * values and uses that information to drive the mechanism to the
   * setpoint, which has previously been defined.
   ******************************************************************/
  public void driveToPoint(){

    double sensorPosition = encoder.getPosition() * kDriveTick2Feet;
    SmartDashboard.getNumber("position", sensorPosition);


    double error = setpoint - sensorPosition;
    double dt = Timer.getFPGATimestamp() - lastTimestamp;


    if (Math.abs(error) < iLimit) {
      errorSum += error * dt;
    }

    double errorRate = (error - lastError) / dt;

    double outputSpeed = kP * error + kI * errorSum + kD * errorRate;

    motor.set(outputSpeed);

  }


  @Override
  public void periodic() {
    // This method will be called once per scheduler run

    SmartDashboard.putNumber("Distance Measured", encoder.getPosition() * kDriveTick2Feet);

  }
}
