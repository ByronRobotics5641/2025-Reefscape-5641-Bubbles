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
import frc.robot.Constants;

public class ElevatorSubsystem extends SubsystemBase {
  /** Creates a new ElevatorSubsystem. */
  //Attatches to SuckerFish/CoralMachine
  // D-pad/pov up = 0°, bottom = 180°
  //two motors one lead, one follow 
  
  boolean isManual = false;//start without PID... change in Elevator PID commands, call setIsManual(false)

  final double kP = .25;
  final double kI = 0;
  final double iLimit = 1;
  final double kD = 0.2;

  final double speedMult = 1.3;

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
  public void eleDriver(double speed, boolean noDown, boolean isManual) 
  {
    this.noDown = noDown;
    if (isManual)
    {
      //System.out.println("isManual");
      if(!noDown && speed > 0) 
      {
        lead.set(0);
        follow.set(0);
      }
      else if(speed < 0 && encoder.getPosition() <= -354) 
      {
        lead.set(0);
        follow.set(0);
      }
      else
      {
        lead.set(speed * .55);
        follow.set(speed * .55);
      }
    }
    else
    {
      setSetpoint();
      heightToPoint();
    }
  }

  public void eleReset() 
  {
    encoder.setPosition(0);
    setSetpoint(0);
  }

  public void eleLift() 
  {
    lead.set(.2);
    follow.set(.2);
  }

  public void eleDown() 
  {
    lead.set(-.2);
    follow.set( -.2);
  }


  public void eleRight() 
  {
    lead.set(.2);
  }

  public void eleLeft() 
  {
    follow.set(.2);
  }


  public void eleStop() 
  {
    lead.set(0);
    lead.set(0);
  }

  public void upCount() 
  {
    //setIsManual(false);
    if(count <= 5)
      count++;
    else
      count = 0;

  }

  public void downCount() {
    //setIsManual(false);
 
    if(count > 0)
      count--;
      
    else
      count = 6;

  }

  public void zeroPID() {
    this.setpoint = 0;
    count = 0;
  }

  /****Buttons****/
  public void Emanip1() {
    count = 1;
  }
  public void Emanip2() {
    count = 2;
  }
  public void Emanip3() {
    count = 3;
  }
  public void Emanip4() {
    count = 4;
  }
  public void Emanip5() {
    count = 5;
  }
  public void Emanip6() {
    count = 6;
  }
  
  
  public void drivePID() {
    this.setpoint = 0;
    //System.out.println("Algae Extract");
  }

  public void setSetpoint() {

    //Elevator L1
    if(count == 1)
      this.setpoint = Constants.L1;

    //Elevator L2
    else if(count == 2) 
      this.setpoint = Constants.L2;

    //Elevator L3
    else if(count == 3)
      this.setpoint = Constants.L3;

    //Elevator Coral Intake
    else if(count == 4)
      this.setpoint = Constants.CORAL_INTAKE;

    //Elevator Algae L2
    else if(count == 5)
      this.setpoint = Constants.ELE_ALGAE_L2;

    //Elevator Algae L3
    else if(count == 6)
      this.setpoint = Constants.ELE_ALGAE_L3;

    //Elevator Resting
    else 
      this.setpoint = 0;

  }
  

  public void setSetpoint(int setpoint) {
    this.count = setpoint;
  }

  public void heightToPoint() {

    double sensorPosition = encoder.getPosition();
    SmartDashboard.getNumber("position", sensorPosition);
    

    if (sensorPosition < -199.125 && kP > 0) { // Assuming kP is positive when going up
      lead.set(0);
      follow.set(0);
      return;  // Stop PID control if height limit is reached
  }

    double error = setpoint - sensorPosition;
    double dt = Timer.getFPGATimestamp() - lastTimestamp;

    //keep I value between 0 and 1
    if (Math.abs(error) < iLimit)
      errorSum += error * dt;

    double errorRate = (error - lastError) / dt;

    double outPutSpeed = kP * error + kI * errorSum + kD * errorRate;



    if(outPutSpeed > 0 && !noDown) {
      lead.set(0);
      follow.set(0);
      encoder.setPosition(0);
    }
    else {
      lead.set(outPutSpeed*speedMult);
      follow.set(outPutSpeed*speedMult);
    }

  }

  
  @Override
  public void periodic() {

      SmartDashboard.putBoolean("Ele Limit", !noDown);
      SmartDashboard.putNumber("Elevator Encoder", encoder.getPosition());  
      SmartDashboard.putBoolean("Manual", isManual);   

  }
}
