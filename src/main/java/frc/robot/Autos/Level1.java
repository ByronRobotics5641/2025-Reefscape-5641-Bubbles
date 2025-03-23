// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Autos;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.Commands.OutCoral;
import frc.robot.Commands.ZeroAngle;
import frc.robot.Commands.MoveCommands.MoveL1;
import frc.robot.subsystems.CoralSubsystem;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class Level1 extends SequentialCommandGroup {
  /** Creates a new Level1. */
  CoralSubsystem coral;

  public Level1(CoralSubsystem coral) {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addCommands(
      new MoveL1(coral).withTimeout(0.1),
      new WaitCommand(1),
      new OutCoral(coral).withTimeout(0.5),
      new WaitCommand(.5),
      new ZeroAngle(coral).withTimeout(.0)
    );
  }
}
