// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

//import java.time.LocalDateTime;

import com.ctre.phoenix6.Utils;
import com.ctre.phoenix6.mechanisms.swerve.LegacySwerveRequest;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
//import com.pathplanner.lib.commands.PathPlannerAuto;
import com.ctre.phoenix6.mechanisms.swerve.LegacySwerveModule.DriveRequestType;
//import com.fasterxml.jackson.databind.ser.std.StdKeySerializers.Default;
//import com.ctre.phoenix6.swerve.SwerveDrivetrain.OdometryThread;
//import com.fasterxml.jackson.databind.util.Named;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.DigitalInput;
//import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.RobotController;
//import edu.wpi.first.wpilibj.PS4Controller.Button;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
//import edu.wpi.first.wpilibj2.command.button.CommandPS5Controller;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.POVButton;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import edu.wpi.first.cameraserver.CameraServer;
import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.AlgaeSubsystem;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.CoralSubsystem;
import frc.robot.subsystems.ElevatorSubsystem;
//import frc.robot.subsystems.LightsSubsystem;
import frc.robot.subsystems.LimeLightSubsystem;
import frc.robot.Autos.AlgaeL2;
import frc.robot.Autos.AlgaeL3;
import frc.robot.Autos.CoralIntake;
import frc.robot.Autos.Level1;
import frc.robot.Autos.Level2;
import frc.robot.Autos.Level3;
import frc.robot.Autos.TestAuto;
//import frc.robot.Commands.AlgaeDriver;
//import frc.robot.Commands.ElevatorDrive;
import frc.robot.Commands.LimelightAlign;
import frc.robot.Commands.MoveCommands.MoveEL2;
import frc.robot.Commands.MoveCommands.MoveL2;

public class RobotContainer {

  /******Declare choosers*******/
  //SendableChooser<Command> autoChooser;//selects autos
  private final SendableChooser<Command> pathChooser;

  /*******controllers*******/
  //private final CommandPS5Controller driver = new CommandPS5Controller(0);
  private final CommandXboxController joystick = new CommandXboxController(0); // Driver joystick
  private final Joystick manip = new Joystick(1); //manip joystick


  private final CommandXboxController m_manipController =
  new CommandXboxController(1);


  /*******subsystems********/
  private final AlgaeSubsystem algaeSubsystem = new AlgaeSubsystem();
  private final CoralSubsystem coralSubsystem = new CoralSubsystem();
  private final ElevatorSubsystem elevatorSubsystem = new ElevatorSubsystem();
  //private final LightsSubsystem lightsSubsystem = new LightsSubsystem();

  /********speed limit variables****/
  double driveSpeed = .6;
  double turtleSpeed = .4;

  /*****Triggers*****/
  DigitalInput angleLimit = new DigitalInput(0);
  Trigger AngleLimit = new Trigger(angleLimit::get);

 // Trigger checkcoralTrigger = new Trigger(coralSubsystem::checkCoral);

  /*DigitalInput eleRight = new DigitalInput(1);
  Trigger EleRight = new Trigger(eleRight::get);*/

  DigitalInput eleLeft = new DigitalInput(2);
  Trigger EleLeft = new Trigger(eleLeft::get);

  DigitalInput noUp = new DigitalInput(1);
  Trigger NoUp = new Trigger(noUp::get);


  Trigger manipCoralIn = new JoystickButton(manip, 3);
  Trigger manipCoralOut = new JoystickButton(manip, 5);

  Trigger manipAlgaeIn = new JoystickButton(manip, 4);
  Trigger manipAlgaeOut = new JoystickButton(manip, 6);

  Trigger manipManualEle = new JoystickButton(manip, 2);
  Trigger manipManualCoral = new JoystickButton(manip, 1);

  //Trigger manipAlgaeUp = new JoystickButton(manip, 12);
  //Trigger manipAlgaeDown = new JoystickButton(manip, 11);

  Trigger l1 = new JoystickButton(manip, 11);
  Trigger l2 = new JoystickButton(manip, 9);
  Trigger l3 = new JoystickButton(manip, 7);
  Trigger coralIntake = new JoystickButton(manip, 12);
  Trigger algaeL2 = new JoystickButton(manip, 10);
  Trigger algaeL3 = new JoystickButton(manip, 8);





  /******pov buttons******/
  POVButton eleHigh = new POVButton(manip, 0);
  POVButton eleLow = new POVButton(manip, 180);
  POVButton coralUp = new POVButton(manip, 0);
  POVButton coralDown = new POVButton(manip, 180);




  //from Commands folder/package------------------------>>subsystem------->>controller input
  //private final AlgaeDriver m_algaeAngle = new AlgaeDriver(algaeSubsystem, m_manipController.getRightY()); //check with control layout
  //private final Command m_algaeAngle = Commands.run(() -> algaeSubsystem.algaeAngle(MathUtil.applyDeadband(m_manipController.getRawAxis(2), 0.5)), algaeSubsystem);
  //private final Command m_algaeLimiter = Commands.run(() -> algaeSubsystem.algaeAngle(MathUtil.applyDeadband(-Math.abs(m_manipController.getRightY()), 0.2), angleLimit.get()), algaeSubsystem);

  private final Command m_algaeDriver = Commands.runOnce(() -> algaeSubsystem.algaeDriver(MathUtil.applyDeadband(m_manipController.getRawAxis(2), 0.5), angleLimit.get()), algaeSubsystem);

  private final Command m_eleDriver = Commands.run(() ->elevatorSubsystem.eleDriver(MathUtil.applyDeadband(m_manipController.getRawAxis(1), 0.4), eleLeft.get(), m_manipController.start().getAsBoolean() || manipManualEle.getAsBoolean()), elevatorSubsystem);
  private final Command m_angleDriver = Commands.run(() ->coralSubsystem.angleDriver(MathUtil.applyDeadband(m_manipController.getRawAxis(1), 0.4),noUp.get(), m_manipController.back().getAsBoolean() || manipManualCoral.getAsBoolean()), coralSubsystem);
  //private final Command m_eleLimiter = Commands.run(() ->elevatorSubsystem.eleDriver(m_manipController.getLeftY()), elevatorSubsystem);

  /******commands******/
 // private final Command m_colors = Commands.runOnce(lightsSubsystem::setAllianceColor, lightsSubsystem);

  private final Command m_coralIn = Commands.runOnce(coralSubsystem::coralIn, coralSubsystem);
  private final Command m_coralOut = Commands.runOnce(coralSubsystem::coralOut, coralSubsystem);
  private final Command m_coralStop = Commands.runOnce(coralSubsystem::coralStop, coralSubsystem);

  //private final Command m_driveUpAngle = Commands.runOnce(coralSubsystem::driveUpAngle, coralSubsystem);
  //private final Command m_driveDownAngle = Commands.runOnce(coralSubsystem::driveDownAngle, coralSubsystem);
  private final Command m_zeroCoral = Commands.runOnce(coralSubsystem::zeroCoral, coralSubsystem);

  //private final Command m_downAngle = Commands.runOnce(coralSubsystem::downAngle, coralSubsystem);
  //private final Command m_upAngle = Commands.runOnce(coralSubsystem::upAngle, coralSubsystem);
  //private final Command m_stopAngle = Commands.runOnce(coralSubsystem::stopAngle, coralSubsystem);
  private final Command m_angleReset = Commands.runOnce(coralSubsystem::angleReset, coralSubsystem);

  private final Command m_manip1 = Commands.runOnce(coralSubsystem::manip1, coralSubsystem);
  private final Command m_manip2 = Commands.runOnce(coralSubsystem::manip2, coralSubsystem);
  private final Command m_manip3 = Commands.runOnce(coralSubsystem::manip3, coralSubsystem);
  private final Command m_manip4 = Commands.runOnce(coralSubsystem::manip4, coralSubsystem);
  private final Command m_manip5 = Commands.runOnce(coralSubsystem::manip5, coralSubsystem);
  private final Command m_manip6 = Commands.runOnce(coralSubsystem::manip6, coralSubsystem);

  private final Command m_Emanip1 = Commands.runOnce(elevatorSubsystem::Emanip1, coralSubsystem);
  private final Command m_Emanip2 = Commands.runOnce(elevatorSubsystem::Emanip2, coralSubsystem);
  private final Command m_Emanip3 = Commands.runOnce(elevatorSubsystem::Emanip3, coralSubsystem);
  private final Command m_Emanip4 = Commands.runOnce(elevatorSubsystem::Emanip4, coralSubsystem);
  private final Command m_Emanip5 = Commands.runOnce(elevatorSubsystem::Emanip5, coralSubsystem);
  private final Command m_Emanip6 = Commands.runOnce(elevatorSubsystem::Emanip6, coralSubsystem);


  private final Command m_startIntake = Commands.runOnce(algaeSubsystem::startIntake, algaeSubsystem);
  private final Command m_reverseIntake = Commands.runOnce(algaeSubsystem::reverseIntake, algaeSubsystem);
  private final Command m_stopIntake = Commands.runOnce(algaeSubsystem::stopIntake, algaeSubsystem);

  //private final Command m_algaeUp = Commands.runOnce(algaeSubsystem::algaeUp, algaeSubsystem);
  //private final Command m_algaeDown = Commands.runOnce(algaeSubsystem::algaeDown, algaeSubsystem);
  //private final Command m_algaeStop = Commands.runOnce(algaeSubsystem::algaeStop, algaeSubsystem);

  private final Command m_eleStop = Commands.runOnce(elevatorSubsystem::eleStop, elevatorSubsystem);
  //private final Command m_eleRight = Commands.runOnce(elevatorSubsystem::eleRight, elevatorSubsystem);
  //private final Command m_eleLeft = Commands.runOnce(elevatorSubsystem::eleLeft, elevatorSubsystem);
  private final Command m_eleReset = Commands.runOnce(elevatorSubsystem::eleReset, elevatorSubsystem);
  private final Command m_eleLift = Commands.runOnce(elevatorSubsystem::eleLift, elevatorSubsystem);
  private final Command m_eleDown = Commands.runOnce(elevatorSubsystem::eleDown, elevatorSubsystem);

  //private final Command m_drivePID = Commands.runOnce(elevatorSubsystem::drivePID, elevatorSubsystem);
  private final Command m_zeroPID = Commands.runOnce(elevatorSubsystem::zeroPID, elevatorSubsystem);

  //for elevator PID
  //private final Command m_eleUpCount = Commands.runOnce(elevatorSubsystem::upCount, elevatorSubsystem);
  //private final Command m_eleDownCount = Commands.runOnce(elevatorSubsystem::downCount, elevatorSubsystem);
  //private final Command m_eleManual = Commands.run(() ->elevatorSubsystem.setIsManual(true), elevatorSubsystem);
  private final Command m_elePID = Commands.run(() ->elevatorSubsystem.setIsManual(false), elevatorSubsystem);

  //private final Command m_bajarCount = Commands.runOnce(coralSubsystem::bajarCount, coralSubsystem);
  //private final Command m_arribaCount = Commands.runOnce(coralSubsystem::arribaCount, coralSubsystem);
  //private final Command m_coralManual = Commands.run(() ->coralSubsystem.setCManual(true), coralSubsystem);
  private final Command m_coralPID = Commands.run(() ->coralSubsystem.setCManual(false), coralSubsystem);
 
 // private final Command m_setalliancecolor = Commands.runOnce(lightsSubsystem::setAllianceColor,lightsSubsystem);
  //private final Command m_defaultcolor = Commands.runOnce(lightsSubsystem::defaultColor,lightsSubsystem);

 //elePID behavior moved to subsystem count methods
  //private final Command m_elePID = Commands.run(() ->elevatorSubsystem.setIsManual(false), elevatorSubsystem);


  //private final Command m_eleNoDown = Commands.runOnce(elevatorSubsystem::eleNoDown, elevatorSubsystem);
  
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
      drivetrain.setDefaultCommand(
        drivetrain.applyRequest(() -> drive.withVelocityX(0.8 *Math.pow((MathUtil.applyDeadband(-joystick.getRawAxis(1), 0.1) * Constants.MaxSpeed),2)* Math.signum(-joystick.getRawAxis(1))) // Drive forward with
                                                                                                                                                                                                // negative Y (forward)
            .withVelocityY(0.8*Math.pow((MathUtil.applyDeadband(-joystick.getRawAxis(0), 0.1) * Constants.MaxSpeed),2)* Math.signum(-joystick.getRawAxis(0)))  // Drive left with negative X (left)
            .withRotationalRate(0.8 *Math.pow((MathUtil.applyDeadband(-joystick.getRawAxis(4), 0.1) * Constants.MaxAngularRate),2)* Math.signum(-joystick.getRawAxis(4))) // Drive counterclockwise with negative X (left)
        ));
        joystick.rightBumper().whileTrue( // Drivetrain will execute this command periodically
        drivetrain.applyRequest(() -> drive.withVelocityX(.2 *Math.pow((turtleSpeed* MathUtil.applyDeadband(-joystick.getRawAxis(1), 0.1) * Constants.MaxSpeed),2)* Math.signum(-joystick.getRawAxis(1))) // Drive forward with
                                                                                                                                                                                                // negative Y (forward)
            .withVelocityY(.2 *Math.pow((turtleSpeed* MathUtil.applyDeadband(-joystick.getRawAxis(0), 0.1) * Constants.MaxSpeed),2)* Math.signum(-joystick.getRawAxis(0)))  // Drive left with negative X (left)
            .withRotationalRate(.2 *Math.pow((turtleSpeed* MathUtil.applyDeadband(-joystick.getRawAxis(4), 0.1) * Constants.MaxAngularRate),2)* Math.signum(-joystick.getRawAxis(4))) // Drive counterclockwise with negative X (left)
        ));
        joystick.rightBumper().whileFalse( // Drivetrain will execute this command periodically
        drivetrain.applyRequest(() -> drive.withVelocityX(0.8 *Math.pow((MathUtil.applyDeadband(-joystick.getRawAxis(1), 0.1) * Constants.MaxSpeed),2)* Math.signum(-joystick.getRawAxis(1))) // Drive forward with
       // negative Y (forward)
            .withVelocityY(0.8 *Math.pow((MathUtil.applyDeadband(-joystick.getRawAxis(0), 0.1) * Constants.MaxSpeed),2)* Math.signum(-joystick.getRawAxis(0)))  // Drive left with negative X (left)
            .withRotationalRate(0.8*Math.pow((MathUtil.applyDeadband(-joystick.getRawAxis(4), 0.1) * Constants.MaxAngularRate),2)* Math.signum(-joystick.getRawAxis(4))) // Drive counterclockwise with negative X (left)
        ));
    joystick.a().whileTrue(drivetrain.applyRequest(() -> brake));
    joystick.y().whileTrue(drivetrain
        .applyRequest(() -> point.withModuleDirection(new Rotation2d(-joystick.getLeftY(), -joystick.getLeftX()))));

    // reset the field-centric heading on left bumper press
    joystick.y().onTrue(drivetrain.runOnce(() -> drivetrain.seedFieldRelative()));

    //joystick.button(3).whileTrue(limelightAlign);

    /*joystick.leftTrigger().onTrue(m_driveUpAngle);
    joystick.leftTrigger().onTrue(m_drivePID);

    joystick.rightTrigger().onTrue(m_driveDownAngle);
    joystick.rightTrigger().onTrue(m_drivePID);*/

    joystick.b().onTrue(m_zeroPID);
    joystick.b().onTrue(m_zeroCoral);
//*/ 


    /*****PS5*****/
        /*drivetrain.setDefaultCommand( drivetrain.applyRequest(() -> drive.withVelocityX(0.2 * Math.pow((MathUtil.applyDeadband(-driver.getRawAxis(1), 0.2) * Constants.MaxSpeed * driveSpeed),3)) // Drive forward with
        // negative Y (forward)
          .withVelocityY(0.2 * Math.pow((MathUtil.applyDeadband(-driver.getRawAxis(0), 0.2)* Constants.MaxSpeed),3))  // Drive left with negative X (left)
            .withRotationalRate(0.2*Math.pow((MathUtil.applyDeadband(-driver.getRawAxis(2), 0.2) * Constants.MaxAngularRate),3)) // Drive counterclockwise with negative X (left)
            )); // Drivetrain will execute this command periodically
          driver.R1().whileFalse(
            drivetrain.applyRequest(() -> drive.withVelocityX(0.2 * Math.pow((MathUtil.applyDeadband(-driver.getRawAxis(1), 0.2) * Constants.MaxSpeed * driveSpeed),3)) // Drive forward with
                                                                                                                                                                                                    // negative Y (forward)
                .withVelocityY(0.2 * Math.pow((MathUtil.applyDeadband(-driver.getRawAxis(0), 0.2) * Constants.MaxSpeed),3))  // Drive left with negative X (left)
                .withRotationalRate(0.2*Math.pow((MathUtil.applyDeadband(-driver.getRawAxis(2), 0.2) * Constants.MaxAngularRate),3)) // Drive counterclockwise with negative X (left)
            ));
          driver.R1().whileTrue( // Drivetrain will execute this command periodically
        drivetrain.applyRequest(() -> drive.withVelocityX(0.2 * Math.pow((turtleSpeed* MathUtil.applyDeadband(-driver.getRawAxis(1), 0.2) * Constants.MaxSpeed),3)) // Drive forward with
                                                                                                                                                                                                // negative Y (forward)
            .withVelocityY(0.2 * Math.pow((turtleSpeed* MathUtil.applyDeadband(-driver.getRawAxis(0), 0.2) * Constants.MaxSpeed),3))  // Drive left with negative X (left)
            .withRotationalRate(0.2*Math.pow((turtleSpeed* MathUtil.applyDeadband(-driver.getRawAxis(4), 0.2) * Constants.MaxAngularRate),3)) // Drive counterclockwise with negative X (left)
        ));


    driver.cross().whileTrue(drivetrain.applyRequest(() -> brake));
    driver.triangle().whileTrue(drivetrain
    .applyRequest(() -> point.withModuleDirection(new Rotation2d(-driver.getLeftY(), -driver.getLeftX()))));

    // reset the field-centric heading on left bumper press
    driver.circle().onTrue(drivetrain.runOnce(() -> drivetrain.seedFieldRelative()));

    //driver.square().whileTrue(limelightAlign);

    driver.L2().whileTrue(m_eleLeft);
    driver.R2().whileTrue(m_eleRight);
//*/ 


    /*****Assigning Stick Values*****/
    algaeSubsystem.setDefaultCommand(m_algaeDriver);// defaults to stick value, can be linked to a button or command trigger later
    elevatorSubsystem.setDefaultCommand(m_eleDriver);
    coralSubsystem.setDefaultCommand(m_angleDriver);
   // lightsSubsystem.setDefaultCommand(m_colors);
    
    /*****Assigning Buttons*****/

    /***Manip Xbox***/
    /*m_manipController.rightBumper().whileTrue(m_coralIn).onFalse(m_coralStop);
    m_manipController.rightTrigger().whileTrue(m_coralOut).onFalse(m_coralStop);
    
    //m_manipController.y().and(NoUp.negate()).whileTrue(m_upAngle).onFalse(m_stopAngle);
    //m_manipController.x().whileTrue(m_downAngle).onFalse(m_stopAngle);
 
    m_manipController.leftBumper().whileTrue(m_startIntake).onFalse(m_stopIntake);
    m_manipController.leftTrigger().whileTrue(m_reverseIntake).onFalse(m_stopIntake);

    //m_manipController.b().onTrue(m_arribaCount);
    //m_manipController.a().onTrue(m_bajarCount);

    /*****Trigger Assign*****/ 
    /*eleHigh.onTrue(m_eleUpCount);
    eleLow.onTrue(m_eleDownCount);
    coralUp.onTrue(m_arribaCount);
    coralDown.onTrue(m_bajarCount);*/

    //checkcoralTrigger.whileTrue(m_setalliancecolor).whileFalse(m_defaultcolor);
    //coralDetect.whileTrue();

    
    //m_manipController.start().whileFalse(m_elePID);

    EleLeft.onTrue(m_eleReset);//*/
    NoUp.onTrue(m_angleReset);

    /***Manip Flight Stick***/
    manipCoralIn.whileTrue(m_coralIn).onFalse(m_coralStop);
    manipCoralOut.whileTrue(m_coralOut).onFalse(m_coralStop);

    manipAlgaeIn.whileTrue(m_startIntake).onFalse(m_stopIntake);
    manipAlgaeOut.whileTrue(m_reverseIntake).onFalse(m_stopIntake);

    manipManualEle.whileFalse(m_elePID);
    manipManualCoral.whileFalse(m_coralPID);

    //manipAlgaeUp.whileTrue(m_algaeUp).onFalse(m_algaeStop);
    //manipAlgaeDown.whileTrue(m_algaeDown).onFalse(m_algaeStop);

    
    l1.onTrue(m_manip1);
    l2.onTrue(m_manip2);
    l3.onTrue(m_manip3);
    coralIntake.onTrue(m_manip4);
    algaeL2.onTrue(m_manip5);  
    algaeL3.onTrue(m_manip6);

    l1.onTrue(m_Emanip1);
    l2.onTrue(m_Emanip2);
    l3.onTrue(m_Emanip3);
    coralIntake.onTrue(m_Emanip4);
    algaeL2.onTrue(m_Emanip5);  
    algaeL3.onTrue(m_Emanip6);

    if (Utils.isSimulation()) {
      drivetrain.seedFieldRelative(new Pose2d(new Translation2d(), Rotation2d.fromDegrees(90)));
    }
    drivetrain.registerTelemetry(logger::telemeterize);

    CameraServer.startAutomaticCapture(0);
    CameraServer.startAutomaticCapture(1);
  }

  public RobotContainer() {


    NamedCommands.registerCommand("Shoot Coral", m_coralOut);
    NamedCommands.registerCommand("Stop Coral", m_coralStop);
    NamedCommands.registerCommand("Load Coral", m_coralIn);

    NamedCommands.registerCommand("Level1", new Level1(coralSubsystem, elevatorSubsystem));
    NamedCommands.registerCommand("Level2", new Level2(coralSubsystem, elevatorSubsystem));
    NamedCommands.registerCommand("Level3", new Level3(coralSubsystem, elevatorSubsystem));
    NamedCommands.registerCommand("CoralIntake", new CoralIntake(coralSubsystem, elevatorSubsystem));
    NamedCommands.registerCommand("AlgaeL2", new AlgaeL2(coralSubsystem, elevatorSubsystem));
    NamedCommands.registerCommand("AlgaeL3", new AlgaeL3(coralSubsystem, elevatorSubsystem));

    /*NamedCommands.registerCommand("Coral Angle Up", m_upAngle);
    NamedCommands.registerCommand("Coral Angle Down", m_downAngle);
    NamedCommands.registerCommand("Coral Angle Stop", m_stopAngle);*/

    NamedCommands.registerCommand("Zero 1", m_zeroCoral);
    NamedCommands.registerCommand("Zero 2", m_zeroPID);

    /*NamedCommands.registerCommand("Algae Intake", m_startIntake);
    NamedCommands.registerCommand("Algae Out", m_reverseIntake);
    NamedCommands.registerCommand("Algae Stop", m_stopIntake);*/

    /*NamedCommands.registerCommand("Algae Angle Up", m_algaeUp);
    NamedCommands.registerCommand("Algae Angle Down", m_algaeDown);
    NamedCommands.registerCommand("Algae Angle Stop", m_algaeStop);*/

    NamedCommands.registerCommand("Ele Up", m_eleLift);
    NamedCommands.registerCommand("Ele Down", m_eleDown);
    NamedCommands.registerCommand("Ele Stop", m_eleStop);

    NamedCommands.registerCommand("L1", new InstantCommand(() -> coralSubsystem.manip1()));
    NamedCommands.registerCommand("L2", new MoveL2(coralSubsystem));
    NamedCommands.registerCommand("L3", m_manip3);
    NamedCommands.registerCommand("Coral", m_manip4);
    NamedCommands.registerCommand("Algae L2", m_manip5);
    NamedCommands.registerCommand("Algae L3", m_manip6);

    NamedCommands.registerCommand("EL1", m_Emanip1);
    NamedCommands.registerCommand("EL2", new MoveEL2(elevatorSubsystem));
    NamedCommands.registerCommand("EL3", m_Emanip3);
    NamedCommands.registerCommand("ECoral", m_Emanip4);
    NamedCommands.registerCommand("EAlgae L2", m_Emanip5);
    NamedCommands.registerCommand("EAlgae L3", m_Emanip6);

    NamedCommands.registerCommand("Coral Default", m_angleDriver);




    pathChooser = AutoBuilder.buildAutoChooser();

    SmartDashboard.putData("path Chooser", pathChooser);
    SmartDashboard.putNumber("Voltage", RobotController.getBatteryVoltage());
   // SmartDashboard.putBoolean("check Coral Trigger", checkcoralTrigger.getAsBoolean());
    

    //autoChooser = new SendableChooser<>();//initialize chooser for autos
    //autoChooser.setDefaultOption("None", Commands.none());//do nothing
    //autoChooser.addOption("TEST: Swerve",testSwerve);     //pit test

    configureBindings();
  }

  public Command getAutonomousCommand() {
    //return autoChooser.getSelected();// gets auto selection from driver station
    return pathChooser.getSelected();

  }
}
