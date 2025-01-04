// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.ctre.phoenix6.Utils;
import com.ctre.phoenix6.mechanisms.swerve.LegacySwerveRequest;
import com.ctre.phoenix6.mechanisms.swerve.LegacySwerveModule.DriveRequestType;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.LimeLightSubsystem;
import frc.robot.Autos.TestAuto;
import frc.robot.Commands.LimelightAlign;

public class RobotContainer {

  /******Declare choosers*******/
  SendableChooser<Command> autoChooser;//selects autos

  /*******controllers*******/
  private final CommandXboxController joystick = new CommandXboxController(0); // My joystick
  
  /*******Set up drive********/
  private final CommandSwerveDrivetrain drivetrain = TunerConstants.DriveTrain; // My drivetrain
  private final LegacySwerveRequest.FieldCentric drive = new LegacySwerveRequest.FieldCentric()
      .withDeadband(Constants.MaxSpeed * 0.2).withRotationalDeadband(Constants.MaxAngularRate * 0.3) // Add a 10% deadband
      .withDriveRequestType(DriveRequestType.OpenLoopVoltage); // I want field-centric
                                                               // driving in open loop
  private final LegacySwerveRequest.SwerveDriveBrake brake = new LegacySwerveRequest.SwerveDriveBrake();
  private final LegacySwerveRequest.PointWheelsAt point = new LegacySwerveRequest.PointWheelsAt();
  private final Telemetry logger = new Telemetry(Constants.MaxSpeed);

  /******initialize additional subsystems*******/
  private final LimeLightSubsystem limelight =  new LimeLightSubsystem(drivetrain);

  /*********Autos************/
  public final Command testSwerve = new TestAuto(drivetrain, drive);

  /********Commands not included with Swerve Builder******/
  public final Command limelightAlign = new LimelightAlign(limelight, drivetrain, drive);



  private void configureBindings() {
    drivetrain.setDefaultCommand( // Drivetrain will execute this command periodically
        drivetrain.applyRequest(() -> drive.withVelocityX(0.2 * Math.pow((joystick.getRawAxis(1) * Constants.MaxSpeed),3)) // Drive forward with
                                                                                                                                                                                                // negative Y (forward)
            .withVelocityY(0.2 * Math.pow((joystick.getRawAxis(0)* Constants.MaxSpeed),3))  // Drive left with negative X (left)
            .withRotationalRate(0.2*Math.pow((-joystick.getRawAxis(2) * Constants.MaxAngularRate),3)) // Drive counterclockwise with negative X (left)
        ));

    joystick.a().whileTrue(drivetrain.applyRequest(() -> brake));
    joystick.b().whileTrue(drivetrain
        .applyRequest(() -> point.withModuleDirection(new Rotation2d(-joystick.getLeftY(), -joystick.getLeftX()))));

    // reset the field-centric heading on left bumper press
    joystick.button(2).onTrue(drivetrain.runOnce(() -> drivetrain.seedFieldRelative()));

    joystick.button(3).whileTrue(limelightAlign);

    if (Utils.isSimulation()) {
      drivetrain.seedFieldRelative(new Pose2d(new Translation2d(), Rotation2d.fromDegrees(90)));
    }
    drivetrain.registerTelemetry(logger::telemeterize);
  }

  public RobotContainer() {
    configureBindings();

    autoChooser = new SendableChooser<>();//initialize chooser for autos
    autoChooser.setDefaultOption("None", Commands.none());//do nothing
    autoChooser.addOption("TEST: Swerve",testSwerve);     //pit test
  }

  public Command getAutonomousCommand() {
    return autoChooser.getSelected();// gets auto selection from driver station
  }
}
