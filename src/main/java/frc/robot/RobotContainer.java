// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.ctre.phoenix6.Utils;
import com.ctre.phoenix6.mechanisms.swerve.LegacySwerveRequest;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.commands.PathPlannerAuto;
import com.ctre.phoenix6.mechanisms.swerve.LegacySwerveModule.DriveRequestType;
//import com.ctre.phoenix6.swerve.SwerveDrivetrain.OdometryThread;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandPS5Controller;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.AlgaeSubsystem;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.CoralSubsystem;
import frc.robot.subsystems.ElevatorSubsystem;
import frc.robot.subsystems.LimeLightSubsystem;
import frc.robot.Autos.TestAuto;
import frc.robot.Commands.AlgaeDriver;
import frc.robot.Commands.ElevatorDrive;
import frc.robot.Commands.LimelightAlign;

public class RobotContainer {

  /******Declare choosers*******/
  //SendableChooser<Command> autoChooser;//selects autos
  private final SendableChooser<Command> pathChooser;

  /*******controllers*******/
  private final CommandPS5Controller driver = new CommandPS5Controller(0);
  private final CommandXboxController joystick = new CommandXboxController(0); // Driver joystick
  private final Joystick manip = new Joystick(1); //manip joystick

  private final CommandXboxController m_manipController =
  new CommandXboxController(1);


  /*******subsystems********/
  private final AlgaeSubsystem algaeSubsystem = new AlgaeSubsystem();
  private final CoralSubsystem coralSubsystem = new CoralSubsystem();
  private final ElevatorSubsystem elevatorSubsystem = new ElevatorSubsystem();

  /*****Triggers*****/
  DigitalInput angleLimit = new DigitalInput(0);
  Trigger AngleLimit = new Trigger(angleLimit::get);



  /******pov buttons******/
  double driveSpeed = .6;
  double turtleSpeed = .4;

  /******commands******/
  private final Command m_coralIn = Commands.runOnce(coralSubsystem::coralIn, coralSubsystem);
  private final Command m_coralOut = Commands.runOnce(coralSubsystem::coralOut, coralSubsystem);
  private final Command m_coralStop = Commands.runOnce(coralSubsystem::coralStop, coralSubsystem);

  //from Commands folder/package------------------------>>subsystem------->>controller input
  private final AlgaeDriver m_algaeAngle = new AlgaeDriver(algaeSubsystem, m_manipController.getRightY()); //check with control layout

  private final ElevatorDrive m_eleDriver = new ElevatorDrive(elevatorSubsystem, m_manipController.getLeftY());

  private final Command m_downAngle = Commands.runOnce(coralSubsystem::downAngle, coralSubsystem);
  private final Command m_upAngle = Commands.runOnce(coralSubsystem::upAngle, coralSubsystem);
  private final Command m_stopAngle = Commands.runOnce(coralSubsystem::stopAngle, coralSubsystem);

  private final Command m_startIntake = Commands.runOnce(algaeSubsystem::startIntake, algaeSubsystem);
  private final Command m_reverseIntake = Commands.runOnce(algaeSubsystem::reverseIntake, algaeSubsystem);
  private final Command m_stopIntake = Commands.runOnce(algaeSubsystem::stopIntake, algaeSubsystem);
  private final Command m_mateAngle = Commands.runOnce(algaeSubsystem::mateAngle, algaeSubsystem);

  private final Command m_eleLift = Commands.runOnce(elevatorSubsystem::eleLift, elevatorSubsystem);
  private final Command m_eleDown = Commands.runOnce(elevatorSubsystem::eleDown, elevatorSubsystem);
  private final Command m_eleStop = Commands.runOnce(elevatorSubsystem::eleStop, elevatorSubsystem);
  
  /*******Set up drive********/
  public final CommandSwerveDrivetrain drivetrain = TunerConstants.DriveTrain; // My drivetrain
  private final LegacySwerveRequest.FieldCentric drive = new LegacySwerveRequest.FieldCentric()
     // .withDeadband(Constants.MaxSpeed * 0.2).withRotationalDeadband(Constants.MaxAngularRate * 0.3) // Add a 10% deadband
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

    /*****Xbox*****/
       drivetrain.setDefaultCommand( // Drivetrain will execute this command periodically
        drivetrain.applyRequest(() -> drive.withVelocityX(0.2 * Math.pow((-joystick.getRawAxis(1) * Constants.MaxSpeed),3)) // Drive forward with
                                                                                                                                                                                                // negative Y (forward)
            .withVelocityY(0.2 * Math.pow((-joystick.getRawAxis(0)* Constants.MaxSpeed),3))  // Drive left with negative X (left)
            .withRotationalRate(0.2*Math.pow((-joystick.getRawAxis(4) * Constants.MaxAngularRate),3)) // Drive counterclockwise with negative X (left)
        ));  
        joystick.rightBumper().whileTrue( // Drivetrain will execute this command periodically
        drivetrain.applyRequest(() -> drive.withVelocityX(0.2 * Math.pow((turtleSpeed*-joystick.getRawAxis(1) * Constants.MaxSpeed),3)) // Drive forward with
                                                                                                                                                                                                // negative Y (forward)
            .withVelocityY(0.2 * Math.pow((turtleSpeed*-joystick.getRawAxis(0)* Constants.MaxSpeed),3))  // Drive left with negative X (left)
            .withRotationalRate(0.2*Math.pow((driveSpeed*-joystick.getRawAxis(4) * Constants.MaxAngularRate),3)) // Drive counterclockwise with negative X (left)
        ));
    joystick.a().whileTrue(drivetrain.applyRequest(() -> brake));
    joystick.y().whileTrue(drivetrain
        .applyRequest(() -> point.withModuleDirection(new Rotation2d(-joystick.getLeftY(), -joystick.getLeftX()))));

    // reset the field-centric heading on left bumper press
    joystick.y().onTrue(drivetrain.runOnce(() -> drivetrain.seedFieldRelative()));

    joystick.button(3).whileTrue(limelightAlign);
//*/ 

    /*****PS5*****/
        /*drivetrain.setDefaultCommand( // Drivetrain will execute this command periodically
         drivetrain.applyRequest(() -> drive.withVelocityX(0.2 * Math.pow((-driver.getRawAxis(1) * Constants.MaxSpeed * driveSpeed),3)) // Drive forward with
                                                                                                                                                                                                // negative Y (forward)
            .withVelocityY(0.2 * Math.pow((-driver.getRawAxis(0)* Constants.MaxSpeed),3))  // Drive left with negative X (left)
            .withRotationalRate(0.2*Math.pow((-driver.getRawAxis(2) * Constants.MaxAngularRate),3)) // Drive counterclockwise with negative X (left)
         ));
          driver.R1().whileTrue( // Drivetrain will execute this command periodically
        drivetrain.applyRequest(() -> drive.withVelocityX(0.2 * Math.pow((turtleSpeed*-driver.getRawAxis(1) * Constants.MaxSpeed),3)) // Drive forward with
                                                                                                                                                                                                // negative Y (forward)
            .withVelocityY(0.2 * Math.pow((turtleSpeed*-driver.getRawAxis(0)* Constants.MaxSpeed),3))  // Drive left with negative X (left)
            .withRotationalRate(0.2*Math.pow((driveSpeed*-driver.getRawAxis(4) * Constants.MaxAngularRate),3)) // Drive counterclockwise with negative X (left)
        ));


    driver.cross().whileTrue(drivetrain.applyRequest(() -> brake));
    driver.triangle().whileTrue(drivetrain
    .applyRequest(() -> point.withModuleDirection(new Rotation2d(-driver.getLeftY(), -driver.getLeftX()))));

    // reset the field-centric heading on left bumper press
    driver.circle().onTrue(drivetrain.runOnce(() -> drivetrain.seedFieldRelative()));

    driver.square().whileTrue(limelightAlign);
//*/ 

    /*****Assigning Stick Values*****/
    algaeSubsystem.setDefaultCommand(m_algaeAngle);// defaults to stick value, can be linked to a button or command trigger later

    elevatorSubsystem.setDefaultCommand(m_eleDriver);
    
    /*****Assigning Buttons*****/

    m_manipController.rightBumper().whileTrue(m_coralIn).onFalse(m_coralStop);
    m_manipController.rightTrigger().whileTrue(m_coralOut).onFalse(m_coralStop);

    m_manipController.back().whileTrue(m_eleLift).onFalse(m_eleStop);
    m_manipController.start().whileTrue(m_eleDown).onFalse(m_eleStop);

    m_manipController.y().whileTrue(m_upAngle).onFalse(m_stopAngle);
    m_manipController.x().whileTrue(m_downAngle).onFalse(m_stopAngle);

    m_manipController.leftBumper().whileTrue(m_startIntake).onFalse(m_stopIntake);
    m_manipController.leftTrigger().whileTrue(m_reverseIntake).onFalse(m_stopIntake);

    /*****Trigger Assign*****/
    AngleLimit.whileTrue(m_mateAngle);



    if (Utils.isSimulation()) {
      drivetrain.seedFieldRelative(new Pose2d(new Translation2d(), Rotation2d.fromDegrees(90)));
    }
    drivetrain.registerTelemetry(logger::telemeterize);


  }

  public RobotContainer() {
    configureBindings();

    NamedCommands.registerCommand("Shoot Coral", m_coralOut);


    pathChooser = AutoBuilder.buildAutoChooser();

    SmartDashboard.putData("path Chooser", pathChooser);

    //autoChooser = new SendableChooser<>();//initialize chooser for autos
    //autoChooser.setDefaultOption("None", Commands.none());//do nothing
    //autoChooser.addOption("TEST: Swerve",testSwerve);     //pit test
  }

  public Command getAutonomousCommand() {
    //return autoChooser.getSelected();// gets auto selection from driver station
    return pathChooser.getSelected();

  }
}
