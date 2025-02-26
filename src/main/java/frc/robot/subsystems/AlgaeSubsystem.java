// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;




import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class AlgaeSubsystem extends SubsystemBase {
  /** Creates a new AlgaeSubsystem. */
  //has two motors one rotation, one intake/deposit
  //BottomFeeder
  private final double kDriveTick2Degrees = 42 / 360 / 100;


    SparkMax arm = new SparkMax(7, MotorType.kBrushless);
    SparkFlex intake = new SparkFlex(2, MotorType.kBrushless);
    RelativeEncoder encoder;

    public AlgaeSubsystem() {
      encoder = arm.getEncoder();
      encoder.setPosition(0);
  
    }

    public void algaeDriver(double speed) {
      intake.set(speed);
    }

    public void algaeUp() {
      arm.set(-.5);
    }
    public void algaeDown() {
      arm.set(-.5);
    }
    public void algaeStop() {
      arm.set(0);
    }

    public void startIntake() {
      intake.set(1);
    }
    public void reverseIntake() {
      intake.set(-1);
    }
    public void stopIntake() {
      intake.set(0);
    }

    public void algaeAngle(double speed, boolean armNo) {
      if(armNo && speed > 0) {
        arm.set(0);
      }
      else {
        arm.set(speed * .4);
      }
    }

    

    /*****Not needed if algaeAngle takes stick value
     
    public void algaeAngleUp(double rightStickvalue) {
      arm.set(-rightStickvalue * .3);
    }
    public void algaeAngleDown(double rightStickvalue) {
      arm.set(rightStickvalue * .3);
    }*/
 
     public void mateAngle() {
      arm.set(0);
     }
  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    //SmartDashboard.getNumber("Algae Encoder", encoder.getPosition() * kDriveTick2Degrees);

    SmartDashboard.putNumber("Algae Encoder", encoder.getPosition() * kDriveTick2Degrees);
  }
}
