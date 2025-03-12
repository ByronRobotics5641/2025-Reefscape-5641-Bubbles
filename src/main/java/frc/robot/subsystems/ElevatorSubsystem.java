// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.spark.SparkMax;

import java.io.OutputStream;

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
  
  boolean isManual = false;//start without PID... change in Elevator PID commands, call setIsManual(false)

  final double kP = 0.25;
  final double kI = 0;
  final double iLimit = 1;
  final double kD = 0.2;

  double setpoint = 0;
  double stopPoint = 0;
  double errorSum = 0;
  double lastTimestamp = 0;
  double lastError = 0;
  int count = 0;

  boolean noDown;

  SparkMax lead = new SparkMax(5, MotorType.kBrushless);
  SparkMax follow = new SparkMax(6, MotorType.kBrushless);
  RelativeEncoder encoder;


  //private final double kDriveTick2Feet = 1 / 64 * 6 * Math.PI / 12; //change for gear ratio, already in rotations

  

  public ElevatorSubsystem() {  
    //Configures a new object
    encoder = lead.getEncoder();
    encoder.setPosition(0);
    errorSum = 0;
    lastError = 0;
    lastTimestamp = Timer.getFPGATimestamp();


  
    //SmartDashboard.putNumber("Elevator Encoder", encoder.getPosition() * kDriveTick2Feet);

    //SmartDashboard.getNumber("Distance Measured", encoder.getPosition() * kDriveTick2Feet);
  }
  
  /******enables/disables PID*******/
  public void setIsManual(boolean isManual){
    //System.out.println("Setting isManual to: " + isManual);  // Debugging line
    this.isManual = isManual;
  }
  // stick value is DoubleSupplier, passed in through command or instant command with ()->m_manipController.getLeftY()
  public void eleDriver(double speed, boolean noDown, boolean isManual) {
    this.noDown = noDown;
    if (isManual)
    {
      System.out.println("isManual");
      if(!noDown && speed > 0) {
        lead.set(0);
        follow.set(0);
      }
      else if(speed < 0 && encoder.getPosition() <= -330) {
        lead.set(0);
        follow.set(0);
      }
      /*else if(speed > 0 && encoder.getPosition() > -20) {
        lead.set(0);
        follow.set(0);
      }*/
      else if(!isManual) {
        lead.set(0);
        follow.set(0);
      }
      else{
        lead.set(speed * .55);
        follow.set(speed * .55);
      }
    }
     else {
      setSetpoint();
      heightToPoint();
    }


    
  }

  public void eleReset() {
    encoder.setPosition(0);
    setSetpoint(0);
  }

  public void eleLift() {
    lead.set(.2);
    follow.set(.2);
  }
  public void eleDown() {
    lead.set(-.2);
    follow.set( -.2);
  }


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
    //setIsManual(false);
    if(count <= 2) {
      count++;
    }
    else {
      count = 0;
    }
  }
  public void downCount() {
    //setIsManual(false);
 
    if(count > 0) {
      count--;
    }
    else {
      count = 3;
    }
  }
  public void setSetpoint() {
    //setIsManual(false);
    if(count == 1) {
      this.setpoint = -77;
      System.out.println("Coral Intake");
    }
    /*else if(count == 2) {
      this.setpoint = -74;
      System.out.println("Algae Extract");
    }*/
    else if(count == 2) {
      this.setpoint = -250;
      System.out.println("L2");
    }
    else if(count == 3) {
      this.setpoint = -326;
      System.out.println("L3");
    }
    else {
      this.setpoint = 0;
      System.out.println("Resting and L1");
    }
    //encoder.setPosition(0); resets position
  }

  public void setSetpoint(int setpoint) {
    this.count = setpoint;
  }

  public void heightToPoint() {

    double sensorPosition = encoder.getPosition();
    SmartDashboard.getNumber("position", sensorPosition);
    

    if (sensorPosition < -330 && kP > 0) { // Assuming kP is positive when going up

      lead.set(0);
      follow.set(0);
      return;  // Stop PID control if height limit is reached
  }

    double error = setpoint - sensorPosition;
    double dt = Timer.getFPGATimestamp() - lastTimestamp;


    if (Math.abs(error) < iLimit) {
      errorSum += error * dt;
    }

    double errorRate = (error - lastError) / dt;

    double outPutSpeed = kP * error + kI * errorSum + kD * errorRate;



    if(outPutSpeed > 0 && !noDown) {
      lead.set(0);
      follow.set(0);
    }
    else {
      lead.set(outPutSpeed);
      follow.set(outPutSpeed);
    }

  }

  
  @Override
  public void periodic() {
    //SmartDashboard.getNumber("Distance Measured", encoder.getPosition() * kDriveTick2Feet);
    // This method will be called once per scheduler run
    /*if (!isManual) {
      setSetpoint();
      heightToPoint();
    }*/

      System.out.println(count);
      //System.out.println(outPutSpeed);
      SmartDashboard.putBoolean("Ele Limit", !noDown);
      SmartDashboard.putNumber("Elevator Encoder", encoder.getPosition());  
      SmartDashboard.putBoolean("Manual", isManual);   

  }
}
