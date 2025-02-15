// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Commands;

import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.CoralSubsystem;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class CoralAxis extends Command {
  /** Creates a new CoralAxis. */
  CoralSubsystem cAngle;
  DoubleSupplier speedSupplier;
  double speed;

  //this constructor handles double input. great for auton... if we aren't using PID
  public CoralAxis(CoralSubsystem cAngle, double speed) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.cAngle = cAngle;
    addRequirements(cAngle);
    this.speed = speed;
  }

  //constructor for handling a Controller input (DoubleSupplier)
  public CoralAxis(CoralSubsystem cAngle, DoubleSupplier speeedSupplier) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.cAngle = cAngle;
    addRequirements(cAngle);
    this.speed = speedSupplier.getAsDouble();
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    cAngle.angleDriver(speed);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    cAngle.angleDriver(speed);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    cAngle.angleDriver(0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
