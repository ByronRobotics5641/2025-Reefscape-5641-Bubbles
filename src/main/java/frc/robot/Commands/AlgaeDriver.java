// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Commands;

import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.AlgaeSubsystem;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class AlgaeDriver extends Command {
  /** Creates a new AlgaeDriver. */
  AlgaeSubsystem algae;
  DoubleSupplier speedSupplier;
  double speed;
  double rightStickValue;
  boolean armNo = false;

  public AlgaeDriver(AlgaeSubsystem algae, double speed, double rightStickValue) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.algae = algae;
    addRequirements(algae);
    this.speed = speed;
    this.rightStickValue = rightStickValue;
  }
  public AlgaeDriver(AlgaeSubsystem algae, double speedSupplier) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.algae = algae;
    addRequirements(algae);
    this.speed = speedSupplier;
    //this.rightStickValue = speedSupplier.getAsDouble();
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    algae.algaeAngle(speed, armNo);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    algae.algaeAngle(speed, armNo);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    algae.algaeAngle(0, armNo);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
