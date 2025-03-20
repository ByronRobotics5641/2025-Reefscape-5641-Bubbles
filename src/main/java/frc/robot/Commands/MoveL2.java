// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Commands;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.subsystems.CoralSubsystem;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class MoveL2 extends InstantCommand {
  /** Creates a new MoveL2. */
  CoralSubsystem coral;
  double lastSensor = 0;
  double sensor = 0;

  public MoveL2(CoralSubsystem coral) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.coral = coral;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    coral.manip2();
  }
}
