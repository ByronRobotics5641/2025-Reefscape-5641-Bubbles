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

public class ElevatorSubsystem extends SubsystemBase {
  /** Creates a new ElevatorSubsystem. */
  //Attatches to SuckerFish/CoralMachine
  // D-pad/pov up = 0°, bottom = 180°
  //two motors one lead, one follow 
  
  boolean isManual = true;//start without PID... change in Elevator PID commands, call setIsManual(false)

  final double kP = 0.4;
  final double kI = 0.0;
  final double iLimit = 1;
  final double kD = 0.08;

  double setpoint = 0;
  double errorSum = 0;
  double lastTimestamp = 0;
  double lastError = 0;
  int count = 0;

  SparkMax lead = new SparkMax(5, MotorType.kBrushless);
  SparkMax follow = new SparkMax(6, MotorType.kBrushless);
  RelativeEncoder encoder;


  private final double kDriveTick2Feet = 1.0 / 21 * 6 * Math.PI / 12;

  public ElevatorSubsystem() {  
    //Configures a new object
    encoder = lead.getEncoder();

    encoder.setPosition(0);
    errorSum = 0;
    lastError = 0;
    lastTimestamp = Timer.getFPGATimestamp();}
  
  /******enables/disables PID*******/
  public void setIsManual(boolean isManual){
    this.isManual = isManual;
  }
  // stick value is DoubleSupplier, passed in through command or instant command with ()->m_manipController.getLeftY()
  public void eleDriver(double speed, boolean noDown) {
    if(!noDown && speed > 0) {
      lead.set(0);
      follow.set(0);
    }
    else{
      lead.set(speed *.4);
      follow.set(speed *.4);
    }
  }

  public void eleReset() {
    encoder.setPosition(0);
    setSetpoint(0);
  }
  /*public void eleLift() {
    lead.set(0);
    follow.set(0);
  }
  public void eleDown() {
    lead.set(0);
    follow.set( 0);
  }*/


  public void eleRight() {
    lead.set(.2);
  }
  public void eleLeft() {
    follow.set(.2);
  }


  public void eleStop() {
    lead.set(0);
    lead.set(0);
  }
  public void upCount() {
    if(count < 3) {
      count++;
    }
    else {
      count = 0;
    }
  }
  public void downCount() {
    if(count > -1) {
      count--;
    }
    else {
      count = 2;
    }
  }
  public void setSetpoint() {
    if(count == 1) {
      this.setpoint = 1;
    }
    else if(count == 2) {
      this.setpoint = 2;
    }
    else {
      this.setpoint = 0;
    }
    //encoder.setPosition(0); resets position... not useful for our subsystems as described
  }
  public void setSetpoint(int setpoint) {
    this.count = setpoint;
  }

  public void heightToPoint() {

    double sensorPosition = encoder.getPosition() * kDriveTick2Feet;
    SmartDashboard.getNumber("position", sensorPosition);


    double error = setpoint - sensorPosition;
    double dt = Timer.getFPGATimestamp() - lastTimestamp;


    if (Math.abs(error) < iLimit) {
      errorSum += error * dt;
    }

    double errorRate = (error - lastError) / dt;

    double outputSpeed = kP * error + kI * errorSum + kD * errorRate;

    lead.set(outputSpeed);
    follow.set(outputSpeed);

  }

  
  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    if (!isManual)
      heightToPoint();
    
  }
}
