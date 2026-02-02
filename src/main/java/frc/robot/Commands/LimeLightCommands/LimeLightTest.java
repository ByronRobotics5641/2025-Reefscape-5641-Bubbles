// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Commands.LimeLightCommands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.LimelightHelpers;
import java.util.function.DoubleSupplier;


/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class LimeLightTest extends Command {
  /** Creates a new LimeLightTest. */
    private final CommandSwerveDrivetrain drivetrain;
    private final String limelightName = "limelight";
    private final double targetDistanceMeters = 2; // 2 meters
    //116 in, 9.6ft max vizability

    private final DoubleSupplier m_driverX;
    private final DoubleSupplier m_driverY;
    private final DoubleSupplier m_driverRot;

    private double lastValidTx = 0;
    private double lastValidDistance = 0;
    private double lastValidTime = 0;

  public LimeLightTest(CommandSwerveDrivetrain drivetrain, DoubleSupplier driverX, DoubleSupplier driverY, DoubleSupplier driverRot) {
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
    LimelightHelpers.PoseEstimate pose = LimelightHelpers.getBotPoseEstimate_wpiBlue(limelightName);
    
    double currentTime = edu.wpi.first.wpilibj.Timer.getFPGATimestamp();
    boolean hasTarget = false;
    double currentDistance = 0;
    double tx = 0;

    if (pose != null && pose.tagCount > 0) {
        LimelightHelpers.RawFiducial closestTag = getClosestFiducial(pose);
        if (closestTag != null) {
            currentDistance = closestTag.distToRobot;
            tx = closestTag.txnc;
            lastValidDistance = currentDistance;
            lastValidTx = tx;
            lastValidTime = currentTime;
            hasTarget = true;
        }
    }

    // If no target found, check if we can use cached data
    if (!hasTarget) {
        if (currentTime - lastValidTime < 0.5 && lastValidTime != 0) {
            // Use cached data
            currentDistance = lastValidDistance;
            tx = lastValidTx;
            System.out.println("Using cached data. Age: " + (currentTime - lastValidTime));
        } else {
            // Too old, stop
            drivetrain.drive(0, 0, 0, false);
            return;
        }
    } else {
        System.out.println("ID: " + (pose.rawFiducials.length > 0 ? pose.rawFiducials[0].id : "null") + " Dist: " + currentDistance + " TX: " + tx);
    }

    double forwardError = currentDistance - targetDistanceMeters;

    // MK4 L2 tuned gains
    double kP_forward = 1.2; // Increased for faster response
    double kP_turn = 0.045;  // Reduced for smoother rotation (was 0.1)

    double forwardCmd = forwardError * kP_forward;
    
    double turnCmd = 0;
    if (Math.abs(tx) > 1.0) { // 1.0 degree deadband
        turnCmd = -tx * kP_turn;
        // Add minimum output to overcome friction if we are outside deadband
        double minTurn = 0.08; 
        if (Math.abs(turnCmd) < minTurn) {
            turnCmd = Math.signum(turnCmd) * minTurn;
        }
    }

    // Limit speeds to keep robot calm but fast enough
    forwardCmd = Math.max(Math.min(forwardCmd, 1.75), -1.75);
    turnCmd = Math.max(Math.min(turnCmd, 1.5), -1.5);

    drivetrain.drive(forwardCmd, 0, turnCmd, false); // Robot centric
  }
  

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    // Driver override: if the driver moves the joystick significantly, end the command
    if (Math.abs(m_driverX.getAsDouble()) > 0.1 || 
        Math.abs(m_driverY.getAsDouble()) > 0.1 || 
        Math.abs(m_driverRot.getAsDouble()) > 0.1) {
        return true;
    }

    double currentTime = edu.wpi.first.wpilibj.Timer.getFPGATimestamp();
    
    // If we've lost the tag for too long, end
    if (currentTime - lastValidTime > 0.5 && lastValidTime != 0) {
        return true;
    }
    
    // If we have never seen a tag, waiting for one... or should we end?
    // Let's assume if we haven't seen one yet, we don't end immediately unless timeout? 
    // Actually, if lastValidTime is 0 (never seen), maybe we shouldn't start driving?
    // But isFinished is called after execute. 
    // Let's keep it simple: if cached data is stale, we end. 
    
    // Logic for "finished" based on position (only if we have recent data)
    double currentDistance = lastValidDistance;
    double tx = lastValidTx;

    double forwardError = currentDistance - targetDistanceMeters;

    // Early finish: if speed is slow and we are close enough, hand back control
    double forwardCmd = forwardError * 1.6; // Same gain as in execute
    double turnCmd = 0;
    if (Math.abs(tx) > 1.0) {
        turnCmd = -tx * 0.04; // Match kP_turn from execute
    }

    boolean distanceClose = Math.abs(forwardError) < 0.1; // 10cm
    boolean centered = Math.abs(tx) < 2.0; // Slightly tighter for "centered" finish
    
    boolean speedSlow = Math.abs(forwardCmd) < 0.1 && Math.abs(turnCmd) < 0.1;

    return (distanceClose && centered) || speedSlow;
  }

  private LimelightHelpers.RawFiducial getClosestFiducial(LimelightHelpers.PoseEstimate pose) {
    if (pose == null || pose.rawFiducials == null || pose.rawFiducials.length == 0) {
        return null;
    }

    LimelightHelpers.RawFiducial closest = pose.rawFiducials[0];
    for (int i = 1; i < pose.rawFiducials.length; i++) {
        if (pose.rawFiducials[i].distToRobot < closest.distToRobot) {
            closest = pose.rawFiducials[i];
        }
    }
    return closest;
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    drivetrain.drive(0, 0, 0, false); // Stop movement when command ends
  }
}
