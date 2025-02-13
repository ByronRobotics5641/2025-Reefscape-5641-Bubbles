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

    private final double kDriveTick2Feet = 1.0 / 21 * 6 * Math.PI / 12;

    final double kP = 0;
    final double kI = 0;
    final double iLimit = 0;
    final double kD = 0;
  
    double setpoint = 0;
    double errorSum = 0;
    double lastTimestamp = 0;
    double lastError = 0;

    RelativeEncoder encoder;


    public void startIntake() {
      intake.set(0);
    }
    public void reverseIntake() {
      intake.set(0);
    }
    public void stopIntake() {
      intake.set(0);
    }
    public void algaeAngleUp(double rightStickvalue) {
      arm.set(-rightStickvalue * 0);
    }
    public void algaeAngleDown(double rightStickvalue) {
      arm.set(rightStickvalue * 0);
    }
    public void pidArm() {

      double sensorPosition = encoder.getPosition() * kDriveTick2Feet;
      SmartDashboard.getNumber("position", sensorPosition);


      double error = setpoint - sensorPosition;
      double dt = Timer.getFPGATimestamp() - lastTimestamp;


      if (Math.abs(error) < iLimit) {
        errorSum += error * dt;
      }

      double errorRate = (error - lastError) / dt;

      double outputSpeed = kP * error + kI * errorSum + kD * errorRate;

      arm.set(outputSpeed);

    }
  

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
