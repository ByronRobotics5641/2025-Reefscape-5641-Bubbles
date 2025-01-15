// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Commands;

import com.ctre.phoenix6.mechanisms.swerve.LegacySwerveRequest.FieldCentric;

import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.LimeLightSubsystem;
import frc.robot.subsystems.LimelightHelpers;


public class LimelightAlign extends Command {

//Create trapezoid Profile so that we don't immediately move at speed
  private static final TrapezoidProfile.Constraints X_CONSTRAINTS = new TrapezoidProfile.Constraints(3,2); //Vel: Mps, Acc: M/s^2
  private static final TrapezoidProfile.Constraints Y_CONSTRAINTS = new TrapezoidProfile.Constraints(3,2); 
  private static final TrapezoidProfile.Constraints OMEGA_CONSTRAINTS = new TrapezoidProfile.Constraints(8,8); //Omega is rotation, fast is okay

  private static final Transform3d TAG_TO_GOAL = new Transform3d(
    new Translation3d(1,0,0),new Rotation3d(0,0,Math.PI)// ROBOT should be 1M away from TARGET, centered with tag, pointing at tag
    );

  LimeLightSubsystem limelight;
  CommandSwerveDrivetrain drivetrain;
  FieldCentric drive;

  

  LimelightHelpers.LimelightResults result;
  private final Pose2d poseProvider;//current estimated pose

  private final ProfiledPIDController xController = new ProfiledPIDController(3, 0, 0, X_CONSTRAINTS);
  private final ProfiledPIDController yController = new ProfiledPIDController(3, 0, 0, Y_CONSTRAINTS);
  private final ProfiledPIDController omegaController = new ProfiledPIDController(2, 0, 0, OMEGA_CONSTRAINTS);

  
  /** Creates a new LimelightAlign. (constructor) */
  public LimelightAlign(LimeLightSubsystem limelight, CommandSwerveDrivetrain drivetrain, FieldCentric drive) {
    //Receive the drive subsystem parts and hold them within the class. These are copies of the drive subsystem parts and the limelight
    this.limelight = limelight;
    this.drivetrain = drivetrain;
    this.drive = drive;
    poseProvider = drivetrain.getState().Pose; //supply pose 

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

    //set command starting position
    var robotPose = drivetrain.getState().Pose;
    omegaController.reset(robotPose.getRotation().getRadians());
    xController.reset(robotPose.getX());
    yController.reset(robotPose.getY());
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {

    //poll the current position
    var robotPose2d = poseProvider;
    var robotPose =
      new Pose3d(
        robotPose2d.getX(),
        robotPose2d.getY(),
        0.0,
        new Rotation3d(0.0,0.0,robotPose2d.getRotation().getRadians())
      );

    LimelightHelpers.LimelightResults result = LimelightHelpers.getLatestResults("limelight");//poll limelight measurment

    //if any tag found
    if (result.valid){

      var camTarget = result.getBotPose3d();

      /*Filtering can be added later. We just want to use any tag found */

      var goalPose = camTarget.transformBy(TAG_TO_GOAL).toPose2d();

      //Drive using these PID calculations
      xController.setGoal(goalPose.getX());
      yController.setGoal(goalPose.getY());
      omegaController.setGoal(goalPose.getRotation().getRadians());

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

      //setrotation speed
      var omegaSpeed = omegaController.calculate(robotPose2d.getRotation().getRadians());
      if (omegaController.atGoal()){
        omegaSpeed=0;
      }

      //post speeds to drivetrain
      drivetrain.setControl(
        drive.withVelocityX(ySpeed)
           .withVelocityY(xSpeed)
           .withRotationalRate(omegaSpeed)
     );
    }

    /* condition if no tag/tag ambiguous here if needed */

    else{
      //this is the same for now
      var xSpeed = xController.calculate(robotPose.getX());
      if (xController.atGoal()){
        //stop
        xSpeed = 0;
      }

      var ySpeed =  yController.calculate(robotPose.getY());
      if (yController.atGoal()){
        ySpeed=0;
      }

      var omegaSpeed = omegaController.calculate(robotPose2d.getRotation().getRadians());
      if (omegaController.atGoal()){
        omegaSpeed=0;
      }

      drivetrain.setControl(
        drive.withVelocityX(ySpeed)
           .withVelocityY(xSpeed)
           .withRotationalRate(omegaSpeed)
     );
    }
    
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    drivetrain.setControl(
      drive.withVelocityX(0)
         .withVelocityY(0)
         .withRotationalRate(0)
   );  }

  // Returns true when the command should end... optional
  @Override
  public boolean isFinished() {
    return false;
  }
}
