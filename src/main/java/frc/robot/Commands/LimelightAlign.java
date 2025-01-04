// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Commands;

import com.ctre.phoenix6.mechanisms.swerve.LegacySwerveRequest.FieldCentric;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.LimeLightSubsystem;

public class LimelightAlign extends Command {

  LimeLightSubsystem limelight;
  CommandSwerveDrivetrain drivetrain;
  FieldCentric drive;

  /** Creates a new LimelightAlign. (constructor) */
  public LimelightAlign(LimeLightSubsystem limelight, CommandSwerveDrivetrain drivetrain, FieldCentric drive) {
    //Receive the drive subsystem parts and hold them within the class. These are copies of the drive subsystem parts and the limelight
    this.limelight = limelight;
    this.drivetrain = drivetrain;
    this.drive = drive;
    // Use addRequirements() here to declare subsystem dependencies.
    
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    limelight.trustLL(true);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {

  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    limelight.trustLL(false);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
