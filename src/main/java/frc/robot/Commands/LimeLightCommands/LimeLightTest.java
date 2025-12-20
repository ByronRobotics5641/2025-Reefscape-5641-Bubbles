// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Commands.LimeLightCommands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.LimelightHelpers;


/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class LimeLightTest extends Command {
  /** Creates a new LimeLightTest. */
    private final CommandSwerveDrivetrain drivetrain;
    private final String limelightName = "limelight";
    private final double targetDistanceMeters = 0.9144; // 3 feet

  public LimeLightTest(CommandSwerveDrivetrain drivetrain) {
    // Use addRequirements() here to declare subsystem dependencies.
        this.drivetrain = drivetrain;
        addRequirements(drivetrain);
    
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {

    // Always use WPI Blue unless you're red side (then change to wpiRed)
    LimelightHelpers.PoseEstimate pose = LimelightHelpers.getBotPoseEstimate_wpiBlue(limelightName);

    // If no pose or no detected tags, do nothing and avoid crashes
    if (pose == null || pose.tagCount == 0) {
        drivetrain.drive(0, 0, 0, true);
        System.out.print("No pose found");
    }

    double forwardDistance = pose.pose.getX(); // meters from tag
    double sidewaysOffset = pose.pose.getY();  // meters left/right from tag center
    System.out.println(forwardDistance + " " + sidewaysOffset);

    double forwardError = forwardDistance - targetDistanceMeters;

    // MK4 L2 tuned gains
    double kP_forward = 0.9;
    double kP_turn = 1.6;

    double forwardCmd = forwardError * kP_forward;
    double turnCmd = sidewaysOffset * kP_turn;

    // Limit speeds to keep robot calm
    forwardCmd = Math.max(Math.min(forwardCmd, 0.7), -0.7);
    turnCmd = Math.max(Math.min(turnCmd, 0.7), -0.7);

    drivetrain.drive(forwardCmd, 0, turnCmd, true);
  }
  


  // Returns true when the command should end.
  @Override
public boolean isFinished() {
    LimelightHelpers.PoseEstimate pose = LimelightHelpers.getBotPoseEstimate_wpiBlue(limelightName);

    if (pose == null || pose.tagCount == 0) return false;

    boolean distanceClose = Math.abs(pose.pose.getX() - targetDistanceMeters) < 0.05;
    boolean centered = Math.abs(pose.pose.getY()) < 0.04;

    return distanceClose && centered;
}

  

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    drivetrain.drive(0, 0, 0, true);

  }
}
