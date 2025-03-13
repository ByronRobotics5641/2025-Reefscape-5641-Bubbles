// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.spark.SparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkLowLevel.MotorType;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class CoralSubsystem extends SubsystemBase {
  /** Creates a new CoralSubsystem. */
  //two motors one axis, one intake/deposit
  //SuckerFeeder

  DigitalInput coralDetect = new DigitalInput(3);

    private final double kDriveTick2Degrees = (1*360) / 100;// rotations to degrees, reduced by 100 times(100:1)

  final double kP = 0.02;
  final double kI = 0.0;
  final double iLimit = 1;
  final double kD = 0.065;

  double setpoint = 0;
  double errorSum = 0;
  double lastTimestamp = 0;
  double lastError = 0;

  int count;

  boolean cManual = false;
  boolean noUp;

  SparkMax rotate = new SparkMax(3, MotorType.kBrushless);
  SparkMax coral = new SparkMax(4, MotorType.kBrushless);
  RelativeEncoder encoder;

  //Configures a new object

   
  public CoralSubsystem() {
    encoder = rotate.getEncoder();
    encoder.setPosition(0);
    errorSum = 0;
    lastError = 0;
    lastTimestamp = Timer.getFPGATimestamp();

  }
 
  public void setCManual(boolean cManual) {
    System.out.println("Setting isManual to: " + cManual);  // Debugging line
    this.cManual = cManual;
  }

  public void coralDriver(double speed) {
    coral.set(speed);
  }
  public void angleDriver(double speed, boolean noUp, boolean cManual) {
    rotate.set(speed);

    this.noUp = noUp;
    if (cManual)
    {
      System.out.println("isManual");
      if(noUp && speed < 0) {
        rotate.set(speed * 0.6);
      }
      else if(!cManual) {
        rotate.set(speed * 0.6);
      }
      else{
        rotate.set(speed * 0.6);
      }
    }
     else {
      setSetpoint();
      angleToPoint();
    }
  }

  /*Coral Angle Without PID*/
  public void downAngle() {
    rotate.set(.3);
  }
  public void upAngle() {
    rotate.set(-.3);
  }
  public void stopAngle() {
    rotate.set(0);
  }

  public void setSetpoint(int setpoint) {
    this.count = setpoint;
  }

  public void angleReset() {
    encoder.setPosition(0);
    setSetpoint(0);
  }

  public void coralIn() {
    coral.set(1);
  }
  public void coralOut() {
    coral.set(-1);
  }
  public void coralStop() {
    coral.set(0);
  }

  public void arribaCount() {
    if(count <= 3) {
      count++;
    }
    else {
      count = 0;
    }
  }
  public void bajarCount() {
 
    if(count >= 0) {
      count--;
    }
    else {
      count = 3;
    }
  }

  public void driveUpAngle() {
    this.setpoint = 0;
  }
  public void driveDownAngle() {
    this.setpoint = 0;
  }

  public void zeroCoral() {
    this.setpoint = 0;
  }

  public void setSetpoint(){
    if(count == 1) {
      this.setpoint = 50;
    }
    else if(count == 2) {
      this.setpoint = 117;
    }
    else if(count == 3) {
      this.setpoint = 95;
    }
    else {
      this.setpoint = 0;
    }
    //encoder.setPosition(0); resets position... not useful for our subsystems as described
  }

  public void angleToPoint(){

    double sensorPosition = encoder.getPosition() * kDriveTick2Degrees;
    SmartDashboard.getNumber("position", sensorPosition);


    double error = setpoint - sensorPosition;
    double dt = Timer.getFPGATimestamp() - lastTimestamp;


    if (Math.abs(error) < iLimit) {
      errorSum += error * dt;
    }

    double errorRate = (error - lastError) / dt;

    double outputSpeed = kP * error + kI * errorSum + kD * errorRate;

    rotate.set(outputSpeed * .7);

  }
  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    /*if(holdAngle) {
      angleToPoint();
    }*/
    SmartDashboard.putNumber("Coral Encoder", encoder.getPosition() * kDriveTick2Degrees);
    SmartDashboard.putBoolean("Coral Sensor", coralDetect.get());
  }
}
