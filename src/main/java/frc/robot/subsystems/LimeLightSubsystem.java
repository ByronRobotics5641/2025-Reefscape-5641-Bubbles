// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Util.RectanglePoseArea;

public class LimeLightSubsystem extends SubsystemBase {
  CommandSwerveDrivetrain drivetrain;
  Alliance alliance; //Do we have an alliance specified?
  private String ll = "limelight";//limelight name
  private Boolean enable = true;//always track position?
  private Boolean trust = false;
  
  private Pose2d botpose;// current reported position according to limelight
  private static final RectanglePoseArea field = new RectanglePoseArea(
  new Translation2d(0.0, 0.0),
  new Translation2d(16.54, 8.02)
  );//How large is the full field in meters?


  /** Creates a new Limelight. */
  public LimeLightSubsystem(CommandSwerveDrivetrain drivetrain) {
    this.drivetrain = drivetrain;
    SmartDashboard.putBoolean("Use Limelight", trust);
    LimelightHelpers.setStreamMode_Standard(ll);;//ll & USB side-by-side... can be set to PnP too
    //
  }

  @Override
  public void periodic() 
  {

    SmartDashboard.putBoolean("Use Limelight", trust); //always show if limelight in use
  
    //constantly check robot position
    if (enable) 
    {
      
      Double targetDistance = LimelightHelpers.getTargetPose3d_CameraSpace(ll).getTranslation().getDistance(new Translation3d());// where is the tag seen now?
      Double confidence = 1 - ((targetDistance - 1) / 6);// how much do we trust limelight measurement (think jitter)
      LimelightHelpers.LimelightResults result = LimelightHelpers.getLatestResults(ll);
      if (result.valid) 
      {
        botpose = LimelightHelpers.getBotPose2d_wpiBlue(ll);// read current robot position if available
        
        if (field.isPoseWithinArea(botpose)) //is the robot in boundry?
        {
          //check if limelight determines distance of 50cm or less |or| if we want it to post anyway |or| there's more than 1 apriltag (megatag)
          if (drivetrain.getState().Pose.getTranslation().getDistance(botpose.getTranslation()) < 0.5 || trust || result.targets_Fiducials.length > 1) 
          {
            //post current pose to drivetrain
            drivetrain.addVisionMeasurement//This is PoseEstimator for CTRE swerve
            (
            botpose,
            Timer.getFPGATimestamp()
                - (result.latency_capture / 1000.0)   //we need to find the latency between ll and RIO
                - (result.latency_pipeline / 1000.0),
            VecBuilder.fill(confidence, confidence, .01)//how much should the robot trust the measurment
            );
          }
        }
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