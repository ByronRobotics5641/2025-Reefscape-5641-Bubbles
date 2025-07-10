// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;




import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.Trigger;

public class AlgaeSubsystem extends SubsystemBase {
  /** Creates a new AlgaeSubsystem. */
  //has two motors one rotation, one intake/deposit
  //BottomFeeder
  private final double kDriveTick2Degrees = (1*360) / 100;// rotations to degrees, reduced by 100 times(100:1)


    SparkMax arm = new SparkMax(7, MotorType.kBrushless);
    SparkFlex intake = new SparkFlex(2, MotorType.kBrushless);
    RelativeEncoder encoder;

    boolean angleLimit;

    public AlgaeSubsystem() {
      encoder = arm.getEncoder();
      encoder.setPosition(0);
  
    }

    public void algaeDriver(double speed, boolean angleLimit) {
      this.angleLimit = angleLimit;
      if(angleLimit && speed > 0) {
        arm.set(speed * 0);
        encoder.setPosition(0);
      }
      else if(speed < 0 && encoder.getPosition() <= -40)
      {
        arm.set(speed * 0);
      }
      else {
        arm.set(speed * .5);
      }
    }

    public void algaeUp() {
      arm.set(-.5);
    }
    public void algaeDown() {
      arm.set(.5);
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

    /*public void algaeAngle(double speed, boolean armNo) {
      if(armNo && speed > 0) {
        arm.set(0);
      }
      else {
        arm.set(speed * .4);
      }
    }*/

    

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

    SmartDashboard.putBoolean("Algae Limit", angleLimit);
    SmartDashboard.putNumber("Algae Encoder", encoder.getPosition());
  }
}
