// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Autos;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.Commands.MoveCommands.MoveEL4;
import frc.robot.Commands.MoveCommands.MoveL4;
import frc.robot.Commands.TopCommands.ShootCoral;
import frc.robot.subsystems.CoralSubsystem;
import frc.robot.subsystems.ElevatorSubsystem;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class CoralIntake extends SequentialCommandGroup {
  /** Creates a new CoralIntake. */
  CoralSubsystem coral;
  ElevatorSubsystem ele;
  public CoralIntake(CoralSubsystem coral, ElevatorSubsystem ele) {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addCommands(
      new MoveEL4(ele).alongWith(new MoveL4(coral)).withTimeout(2),
      new WaitCommand(.1),
      new ShootCoral(coral).withTimeout(20)
    );
  }
}
