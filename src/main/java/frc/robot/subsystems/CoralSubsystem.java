// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.spark.SparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkLowLevel.MotorType;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class CoralSubsystem extends SubsystemBase {
  /** Creates a new CoralSubsystem. */
  //two motors one axis, one intake/deposit
  //SuckerFeeder
    private final double kDriveTick2Degrees = 360 / 42 * 100;

  final double kP = 0.4;
  final double kI = 0.0;
  final double iLimit = 1;
  final double kD = 0.08;

  double setpoint = 0;
  double errorSum = 0;
  double lastTimestamp = 0;
  double lastError = 0;

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
 

  public void coralDriver(double speed) {
    coral.set(speed);
  }
  public void angleDriver(double speed) {
    rotate.set(speed);
  }

  public void downAngle() {
    rotate.set(.3);
  }
  public void upAngle() {
    rotate.set(-.3);
  }
  public void stopAngle() {
    rotate.set(0);
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
  public void setSetpoint(double setpoint){
    this.setpoint = setpoint;
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

    rotate.set(outputSpeed);

  }
  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
