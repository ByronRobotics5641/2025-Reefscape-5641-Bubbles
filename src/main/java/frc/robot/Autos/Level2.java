// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Autos;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.Commands.MoveCommands.MoveEL2;
import frc.robot.Commands.MoveCommands.MoveL2;
import frc.robot.Commands.TopCommands.OutCoral;
import frc.robot.Commands.TopCommands.ZeroAngle;
import frc.robot.Commands.TopCommands.ZeroEle;
import frc.robot.subsystems.CoralSubsystem;
import frc.robot.subsystems.ElevatorSubsystem;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class Level2 extends SequentialCommandGroup {
  /** Creates a new CoralAuto1. */
  CoralSubsystem coral;
  ElevatorSubsystem ele;

  public Level2(CoralSubsystem coral, ElevatorSubsystem ele) {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addCommands(
      new MoveEL2(ele).withTimeout(1),
      new WaitCommand(.5),
      new MoveL2(coral).withTimeout(1),
      new WaitCommand(1),
      new OutCoral(coral).withTimeout(.5),
      new WaitCommand(.5),
      new ZeroEle(ele).alongWith(new ZeroAngle(coral)).withTimeout(1)
    );
  }
}
