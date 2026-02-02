// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Commands.LimeLightCommands;

import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.LimelightHelpers;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class LimeLightStare extends Command {
  /** Creates a new LimeLightStare. */
    private final CommandSwerveDrivetrain drivetrain;
    private final String limelightName = "limelight";
    
    private final DoubleSupplier m_driverX;
    private final DoubleSupplier m_driverY;

    private final DoubleSupplier m_driverRot;

    private double lastValidTx = 0;
    private double lastValidDistance = 0;
    private double lastValidTime = 0;


  public LimeLightStare(CommandSwerveDrivetrain drivetrain, DoubleSupplier driverX, DoubleSupplier driverY, DoubleSupplier driverRot) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.drivetrain = drivetrain;
    this.m_driverX = driverX;
    this.m_driverY = driverY;
    this.m_driverRot = driverRot;

    addRequirements(drivetrain);

  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    lastValidTime = 0; // Reset on init
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    double currentTime = edu.wpi.first.wpilibj.Timer.getFPGATimestamp();
    boolean hasTarget = false;
    double currentDistance = 0;
    double tx = 0;

      // If no target found, check if we can use cached data
    if (!hasTarget) {
        if (currentTime - lastValidTime < 0.5 && lastValidTime != 0) {
            // Use cached data
            currentDistance = lastValidDistance;
            tx = lastValidTx;
            System.out.println("Using cached data. Age: " + (currentTime - lastValidTime));
        } else {
            // Too old, stop
            drivetrain.drive(0, 0, 0, true);
            return;
        }
    } else {
        System.out.println("ID: " + " Dist: " + currentDistance + " TX: " + tx);
    }

    double kP_turn = 0.045;  // Reduced for smoother rotation (was 0.1)

    double turnCmd = 0;

    if (Math.abs(tx) > 1.0) { // 1.0 degree deadband
      turnCmd = -tx * kP_turn;
      // Add minimum output to overcome friction if we are outside deadband
      double minTurn = 0.08; 
      if (Math.abs(turnCmd) < minTurn) {
          turnCmd = Math.signum(turnCmd) * minTurn;
      }
  }

  turnCmd = Math.max(Math.min(turnCmd, 1.5), -1.5);

  drivetrain.drive(m_driverX.getAsDouble(), m_driverY.getAsDouble(), turnCmd, true);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    if (Math.abs(m_driverRot.getAsDouble()) > 0.1) {
      return true;
    }
    double currentTime = edu.wpi.first.wpilibj.Timer.getFPGATimestamp();
    
    // If we've lost the tag for too long, end
    if (currentTime - lastValidTime > 0.5 && lastValidTime != 0) {
        return true;
    }

    double tx = lastValidTx;

    double turnCmd = 0;
    if (Math.abs(tx) > 1.0) {
        turnCmd = -tx * 0.04; // Match kP_turn from execute
    }
    boolean centered = Math.abs(tx) < 2.0; // Slightly tighter for "centered" finish
    boolean speedSlow = Math.abs(turnCmd) < 0.1;


    return (centered) || speedSlow;

  }

    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {
      drivetrain.drive(0, 0, 0, true); // Stop movement when command ends

    }
}
