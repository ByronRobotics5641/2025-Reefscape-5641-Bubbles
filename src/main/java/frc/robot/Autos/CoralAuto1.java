// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Autos;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.Commands.MoveEL2;
import frc.robot.Commands.MoveL2;
import frc.robot.Commands.OutCoral;
import frc.robot.Commands.ShootCoral;
import frc.robot.Commands.ZeroAngle;
import frc.robot.Commands.ZeroEle;
import frc.robot.subsystems.CoralSubsystem;
import frc.robot.subsystems.ElevatorSubsystem;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class CoralAuto1 extends SequentialCommandGroup {
  /** Creates a new CoralAuto1. */
  CoralSubsystem coral;
  ElevatorSubsystem ele;

  public CoralAuto1(CoralSubsystem coral, ElevatorSubsystem ele) {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addCommands(
      new MoveEL2(ele).alongWith(new MoveL2(coral)).withTimeout(3),
      new WaitCommand(3),
      new OutCoral(coral).withTimeout(2),
      new WaitCommand(2),
      new ZeroEle(ele).alongWith(new ZeroAngle(coral)).withTimeout(2)
    );
  }
}
