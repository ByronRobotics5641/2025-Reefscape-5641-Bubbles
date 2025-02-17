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
  public AlgaeSubsystem() {}
    SparkMax arm = new SparkMax(1, MotorType.kBrushless);
    SparkFlex intake = new SparkFlex(2, MotorType.kBrushless);


    public void algaeDriver(double speed) {
      intake.set(speed);
    }

    


    public void startIntake() {
      intake.set(1);
    }
    public void reverseIntake() {
      intake.set(1);
    }
    public void stopIntake() {
      intake.set(0);
    }

    public void algaeAngle(double speed) {
      arm.set(speed);
    }
    public void algaeAngleUp(double rightStickvalue) {
      arm.set(-rightStickvalue * .3);
    }
    public void algaeAngleDown(double rightStickvalue) {
      arm.set(rightStickvalue * .3);
    }
 
     public void mateAngle() {
      arm.set(0);
     }
  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
