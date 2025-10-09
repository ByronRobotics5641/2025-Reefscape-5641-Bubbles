// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.ColorSensorV3;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkLowLevel.MotorType;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.I2C;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class CoralSubsystem extends SubsystemBase {
  /** Creates a new CoralSubsystem. */
  //two motors one axis, one intake/deposit
  //SuckerFeeder

  /////////////
  private final I2C.Port i2cPort = I2C.Port.kOnboard;//use I2C port of RIO
  private final ColorSensorV3 colorSensor = new ColorSensorV3(i2cPort);//v3 color sensor
  /////////////

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
  SparkFlex coral = new SparkFlex(4, MotorType.kBrushless);
  RelativeEncoder encoder;

  //Configures a new object

   
  public CoralSubsystem() {
    encoder = rotate.getEncoder();
    encoder.setPosition(0);
    errorSum = 0;
    lastError = 0;
    lastTimestamp = Timer.getFPGATimestamp();

  }

  //treat proximity like limit switch
  public boolean checkCoral(){
    //use value when nothing is in SF
    if (colorSensor.getProximity() > 150){
      return true;
    }
    else{
      return false;
    }
  }

 
  public void setCManual(boolean cManual) {
    System.out.println("Setting isManual to: " + cManual);  // Debugging line
    this.cManual = cManual;
  }

  public void coralDriver(double speed) {
    if(checkCoral() && speed < 0) {
      coral.set(0);
    }
    else {
      coral.set(speed);
    }
  }
  public void angleDriver(double speed, boolean noUp, boolean cManual) {
    rotate.set(speed);

    this.noUp = noUp;
    if (cManual)
    {
      System.out.println("isManual");
      if(noUp && speed < 0) {
        rotate.set(speed * 0);
      }
      else{
        rotate.set(speed * 0.8);
      }
    }
     else if(!cManual) {
      setSetpoint();
      angleToPoint();
     }
     else {
      rotate.set(speed * 0);
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
    if(checkCoral()) {
      coral.set(0);
    }
    else {
      coral.set(-1);
    }
  }
  public void coralStop() {
    coral.set(0);
  }

  public void arribaCount() {
    if(count <= 5) {
      count++;
    }
    else {
      count = 0;
    }
  }
  public void bajarCount() {
 
    if(count > 0) {
      count--;
    }
    else {
      count = 7;
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
    count = 0;
  }

  /****Buttons****/
  public void manip1() {
    count = 1;
  }
  public void manip2() {
    count = 2;
  }
  public void manip3() {
    count = 3;
  }
  public void manip4() {
    count = 4;
  }
  public void manip5() {
    count = 5;
  }
  public void manip6() {
    count = 6;
  }
  public void manip7() {
    count = 7;
  }
  
  
  public void setSetpoint(){
    if(count == 1) { //L1
      this.setpoint = Constants.CL1;
     // System.out.println("Coral L1");
    }
    else if(count == 2) { //L2
      this.setpoint = Constants.CL2;
    //  System.out.println("Coral L2");
    }
    else if(count == 3) { //L3
      this.setpoint = Constants.CL3;
      //System.out.println("Coral L3");
    }
    else if(count == 7) {
      this.setpoint = Constants.CL4;
    }
    else if(count == 4) { //Coral Intake
      this.setpoint = Constants.C_CORAL_INTAKE;
     // System.out.println("Coral Coral Intake");
    }
    else if(count == 5) { //Algae L2
      this.setpoint = Constants.C_ALGAE_L2;
      //System.out.println("Coral Algae L2");
    }
    else if(count == 6) { //Algae L3
      this.setpoint = Constants.C_ALGAE_L3;
      //System.out.println("Coral Algae L3");
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

    //System.out.println("SF Count: "+ count);

    SmartDashboard.putNumber("SF Count", count);

    SmartDashboard.putNumber("Coral Encoder", encoder.getPosition() * kDriveTick2Degrees);
    SmartDashboard.putNumber("Color Sensor", colorSensor.getProximity());
    SmartDashboard.putBoolean("colorboolean", checkCoral());
    SmartDashboard.putBoolean("coral limit", noUp);
  }
}
