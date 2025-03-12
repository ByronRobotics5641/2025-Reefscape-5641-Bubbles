// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Commands;

import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.ElevatorSubsystem;


//for the weirdos who dont like PID

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class ElevatorDrive extends Command {
  /** Creates a new ElevatorDrive. */
  ElevatorSubsystem elevator;
  DoubleSupplier speedSupplier;
  double speed;
  boolean noDown = false;
  boolean isManual = false;

  public ElevatorDrive(ElevatorSubsystem elevator, double speed) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.elevator = elevator;
    addRequirements(elevator);
    this.speed = speed;
  }
  public ElevatorDrive(ElevatorSubsystem elevator, DoubleSupplier speedSupplier) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.elevator = elevator;
    addRequirements(elevator);
    this.speed = speedSupplier.getAsDouble();
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    elevator.eleDriver(speed, !noDown, isManual);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    elevator.eleDriver(speed, !noDown, isManual);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    elevator.eleDriver(0, !noDown, isManual);
  }
  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
