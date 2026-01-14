// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.



package frc.robot.subsystems;

import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Util.RectanglePoseArea;

public class LimeLightSubsystem extends SubsystemBase {
  CommandSwerveDrivetrain drivetrain;
  Alliance alliance;
  private String ll = "limelight";
  private Boolean enable = false;
  private Boolean trust = false;
  private int fieldError = 0;
  private int distanceError = 0;
  private Pose2d botpose;
  private static final RectanglePoseArea field =
        new RectanglePoseArea(new Translation2d(0.0, 0.0), new Translation2d(16.54, 8.02));

  /** Creates a new Limelight. */
  public LimeLightSubsystem(CommandSwerveDrivetrain drivetrain) {
    this.drivetrain = drivetrain;
    SmartDashboard.putNumber("Field Error", fieldError);
    SmartDashboard.putNumber("Limelight Error", distanceError);
  }

  @Override
  public void periodic() {
    if (enable) {
      // MegaTag2: Send robot orientation to Limelight
      var pose = drivetrain.getState().Pose;
      var speeds = drivetrain.getState().speeds;
      
      LimelightHelpers.SetRobotOrientation(ll, 
          pose.getRotation().getDegrees(), 
          edu.wpi.first.math.util.Units.radiansToDegrees(speeds.omegaRadiansPerSecond), 
          0, 0, 0, 0);

      // Use MegaTag2 Pose estimate for WPILib Blue alliance
      LimelightHelpers.PoseEstimate result = LimelightHelpers.getBotPoseEstimate_wpiBlue_MegaTag2(ll);
      
      if (result != null && result.tagCount > 0) {
          // Calculate individual confidence based on distance
          double targetDistance = result.avgTagDist;
          double confidence = 1 - ((targetDistance - 1) / 6);
          confidence = Math.max(0.1, Math.min(1.0, confidence));

          // Add vision measurement to drivetrain
          drivetrain.addVisionMeasurement(
              result.pose,
              result.timestampSeconds,
              VecBuilder.fill(confidence, confidence, .01));
      }
    }
  }

  public void setAlliance(Alliance alliance) {
    this.alliance = alliance;
  }

  public void useLimelight(boolean enable) {
    this.enable = enable;
  }

  public void trustLL(boolean trust) {
    this.trust = trust;
  }
}