// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Autos;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.Commands.OutCoral;
import frc.robot.Commands.MoveCommands.MoveEL6;
import frc.robot.Commands.MoveCommands.MoveL6;
import frc.robot.subsystems.CoralSubsystem;
import frc.robot.subsystems.ElevatorSubsystem;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class AlgaeL3 extends SequentialCommandGroup {
  /** Creates a new AlgaeL3. */
  CoralSubsystem coral;
  ElevatorSubsystem ele;

  public AlgaeL3(CoralSubsystem coral, ElevatorSubsystem ele) {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addCommands(
      new MoveEL6(ele).withTimeout(1),
      new WaitCommand(.5),
      new MoveL6(coral).alongWith(new OutCoral(coral)).withTimeout(8)
    );
  }
}
