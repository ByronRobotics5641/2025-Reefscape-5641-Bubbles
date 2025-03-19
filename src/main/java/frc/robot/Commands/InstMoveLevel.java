// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Commands;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.subsystems.CoralSubsystem;
import frc.robot.subsystems.ElevatorSubsystem;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class InstMoveLevel extends InstantCommand {
  CoralSubsystem coral;
  ElevatorSubsystem ele;
  int setPoint;
    public InstMoveLevel(int count, CoralSubsystem coral, ElevatorSubsystem ele) {
    this.coral = coral;
    this.ele = ele;
    setPoint=count;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(coral,ele);
    }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    ele.setSetpoint(setPoint);
    coral.setSetpoint(setPoint);
  }
}
