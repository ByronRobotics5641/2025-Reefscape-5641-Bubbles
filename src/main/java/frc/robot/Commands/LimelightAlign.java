// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Commands;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;

import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile;

import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import com.ctre.phoenix6.mechanisms.swerve.LegacySwerveRequest.FieldCentric;
import frc.robot.subsystems.LimeLightSubsystem;
import frc.robot.subsystems.LimelightHelpers;


public class LimelightAlign extends Command {

//Create trapezoid Profile so that we don't immediately move at speed
  private static final TrapezoidProfile.Constraints X_CONSTRAINTS = new TrapezoidProfile.Constraints(.7,.1); //Vel: Mps, Acc: M/s^2
  private static final TrapezoidProfile.Constraints Y_CONSTRAINTS = new TrapezoidProfile.Constraints(.7,.1); 
  private static final TrapezoidProfile.Constraints OMEGA_CONSTRAINTS = new TrapezoidProfile.Constraints(1.7,.50); //Omega is rotation, fast is okay

  private static final Transform3d TAG_TO_GOAL = new Transform3d(
    new Translation3d(1.5,0,0),
    new Rotation3d(0,0,0)// ROBOT should be 1M away from TARGET, centered with tag, pointing at tag
    );

  LimeLightSubsystem limelight;
  CommandSwerveDrivetrain drivetrain;
  FieldCentric drive;

  
  
  LimelightHelpers.LimelightResults result;

  private final ProfiledPIDController xController = new ProfiledPIDController(3, 0, 0, X_CONSTRAINTS);
  private final ProfiledPIDController yController = new ProfiledPIDController(3, 0, 0, Y_CONSTRAINTS);
  private final ProfiledPIDController omegaController = new ProfiledPIDController(2, 0., 0, OMEGA_CONSTRAINTS);

  
  /** Creates a new LimelightAlign. (constructor) */
  public LimelightAlign(LimeLightSubsystem limelight, CommandSwerveDrivetrain drivetrain, FieldCentric drive) {
    //Receive the drive subsystem parts and hold them within the class. These are copies of the drive subsystem parts and the limelight
    this.limelight = limelight;
    this.drivetrain = drivetrain;
    this.drive = drive;



    //set where it can stop trying to line up
    xController.setTolerance(0.2);
    yController.setTolerance(0.2);
    omegaController.setTolerance(Units.degreesToRadians(3));

    omegaController.enableContinuousInput(-Math.PI, Math.PI);//range of -180 to 180 degrees

   

    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(drivetrain);
    
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    //point at target
    //drivetrain.seedFieldRelative();
   // System.out.println("Zero facing target");
    //start using LL
    limelight.useLimelight(true);
    limelight.trustLL(true);

    //set command starting position
    //Pose2d poseProvider = drivetrain.getState().Pose;
    var robotPose = LimelightHelpers.getTargetPose3d_CameraSpace("limelight");
    omegaController.reset(robotPose.getRotation().getAngle());
    xController.reset(robotPose.getX());
    yController.reset(robotPose.getY());

    System.out.println(robotPose);

  
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {

    //poll the current position
    //Pose2d poseProvider = drivetrain.getState().Pose;
    var robotPose2d =LimelightHelpers.getTargetPose3d_CameraSpace("limelight");
    var robotPose =
      new Pose3d(
        robotPose2d.getX(),
        robotPose2d.getY(),
        0.0,
        new Rotation3d(0.0,0.0,robotPose2d.getRotation().getAngle())
      );

    //LimelightHelpers.LimelightResults result = LimelightHelpers.getLatestResults("limelight");//poll limelight measurment

    //if any tag found
    if (LimelightHelpers.getTV("limelight")){
      System.out.println("target found");
      var camTarget = LimelightHelpers.getTargetPose3d_CameraSpace("limelight");

      /*Filtering can be added later. We just want to use any tag found */

      var goalPose = camTarget.transformBy(TAG_TO_GOAL).toPose2d();

      //Drive to the given target using these PID calculations
      xController.setGoal(TAG_TO_GOAL.getX());
      yController.setGoal(TAG_TO_GOAL.getY());
      omegaController.setGoal(TAG_TO_GOAL.getRotation().getAngle());

      System.out.println("Goal pose: "+goalPose);

      //set speed for side to side
      var xSpeed = xController.calculate(robotPose.getX());
      if (xController.atGoal()){
        //stop
        xSpeed = 0;
      }

      //set speed for forward/backward
      var ySpeed =  yController.calculate(robotPose.getY());
      if (yController.atGoal()){
        ySpeed=0;
      }

      //set rotation speed
      var omegaSpeed = omegaController.calculate(robotPose2d.getRotation().getAngle());
      if (omegaController.atGoal()){
        omegaSpeed=0;
      }

      System.out.println("current speed output: "+xSpeed + ", "+ySpeed +", "+omegaSpeed);

      //post speeds to drivetrain
      drivetrain.setControl(
        drive.withVelocityX(xSpeed*Constants.MaxSpeed)
           //.withVelocityY(ySpeed*Constants.MaxSpeed)
           //.withRotationalRate(omegaSpeed)
     );
    }

    /* condition if no tag/tag ambiguous here if needed */

    /*else{
      System.out.println("no target found");
      drivetrain.setControl(
      drive.withVelocityX(0)
         .withVelocityY(0)
         .withRotationalRate(0)
   );  
    }*/
    
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    drivetrain.setControl(
      drive.withVelocityX(0)
         .withVelocityY(0)
         .withRotationalRate(0)
   );

   //stop using LL
    limelight.trustLL(false);
    limelight.useLimelight(false);
  }

  // Returns true when the command should end... optional
  @Override
  public boolean isFinished() {
    return false;
  }
}
