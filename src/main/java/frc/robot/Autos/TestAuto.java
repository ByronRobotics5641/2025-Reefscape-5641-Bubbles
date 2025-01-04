// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Autos;

import com.ctre.phoenix6.mechanisms.swerve.LegacySwerveRequest.FieldCentric;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.CommandSwerveDrivetrain;
// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class TestAuto extends SequentialCommandGroup {
  /** Creates a new DriveForwardAuto. */
  public TestAuto(CommandSwerveDrivetrain drivetrain, FieldCentric drive) {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    final double MaxSpeed = 3.25; //Meters per Second
    final double MaxAngularRate = .825 * Math.PI; //Radians
    addCommands(
      //forward, 3 seconds
      drivetrain.applyRequest(()->drive.withVelocityX(-0.60*MaxSpeed).withVelocityY(0).withRotationalRate(0)).withTimeout(5),
      //stop,1
      drivetrain.applyRequest(()->drive.withVelocityX(0).withVelocityY(0).withRotationalRate(0)).withTimeout(1),
      //right,3
      drivetrain.applyRequest(()->drive.withVelocityX(0).withVelocityY(0.60*MaxSpeed).withRotationalRate(0)).withTimeout(5),
      //stop,1
      drivetrain.applyRequest(()->drive.withVelocityX(0).withVelocityY(0).withRotationalRate(0)).withTimeout(1),
      //spin,3
      drivetrain.applyRequest(()->drive.withVelocityX(0).withVelocityY(0).withRotationalRate(0.8*MaxAngularRate)).withTimeout(1),
      //stop
      drivetrain.applyRequest(()->drive.withVelocityX(0).withVelocityY(0).withRotationalRate(0)).withTimeout(1),
      //forward to reset wheels
      drivetrain.applyRequest(()->drive.withVelocityX(-.60*MaxSpeed).withVelocityY(0).withRotationalRate(0)).withTimeout(1),
      //stop
      drivetrain.applyRequest(()->drive.withVelocityX(0).withVelocityY(0).withRotationalRate(0)).withTimeout(0.1)
      
      );
  }
}
