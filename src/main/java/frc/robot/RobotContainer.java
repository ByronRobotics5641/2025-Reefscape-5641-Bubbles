// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.ctre.phoenix6.Utils;
import com.ctre.phoenix6.mechanisms.swerve.LegacySwerveRequest;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import com.ctre.phoenix6.mechanisms.swerve.LegacySwerveModule.DriveRequestType;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.filter.LinearFilter;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.POVButton;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import edu.wpi.first.cameraserver.CameraServer;

import frc.robot.subsystems.AlgaeSubsystem;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.CoralSubsystem;
import frc.robot.subsystems.ElevatorSubsystem;
import frc.robot.subsystems.LimeLightSubsystem;
import frc.robot.subsystems.generated.TunerConstants;
import frc.robot.Autos.*;
import frc.robot.Commands.LimeLightCommands.LimeLightStare;
import frc.robot.Commands.LimeLightCommands.LimeLightTest;
import frc.robot.Commands.MoveCommands.MoveEL2;
import frc.robot.Commands.MoveCommands.MoveL2;

public class RobotContainer {

  /******Declare choosers*******/
  private final SendableChooser<Command> pathChooser;

  /*******controllers*******/
  private final CommandXboxController joystick = new CommandXboxController(0); // Driver joystick
  private final Joystick manip = new Joystick(1); //manip joystick
  private final CommandXboxController m_manipController = new CommandXboxController(1);

  /*******subsystems********/
  private final AlgaeSubsystem algaeSubsystem = new AlgaeSubsystem();
  private final CoralSubsystem coralSubsystem = new CoralSubsystem();
  private final ElevatorSubsystem elevatorSubsystem = new ElevatorSubsystem();

  /********speed & safety variables****/
  double driveSpeed = 0.8;
  double turtleSpeed = 0.4; 
  
  private final LinearFilter voltageFilter = LinearFilter.movingAverage(50);
  private double currentX = 0.0;
  private double currentY = 0.0;
  private double currentRot = 0.0;

  /*****Triggers*****/
  DigitalInput angleLimit = new DigitalInput(0);
  Trigger AngleLimit = new Trigger(angleLimit::get);

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

  /******commands******/
  private final Command m_algaeDriver = Commands.runOnce(() -> algaeSubsystem.algaeDriver(MathUtil.applyDeadband(m_manipController.getRawAxis(2), 0.5), angleLimit.get()), algaeSubsystem);
  private final Command m_eleDriver = Commands.run(() ->elevatorSubsystem.eleDriver(MathUtil.applyDeadband(m_manipController.getRawAxis(1), 0.4), eleLeft.get(), m_manipController.start().getAsBoolean() || manipManualEle.getAsBoolean()), elevatorSubsystem);
  private final Command m_angleDriver = Commands.run(() ->coralSubsystem.angleDriver(MathUtil.applyDeadband(m_manipController.getRawAxis(1), 0.4),noUp.get(), m_manipController.back().getAsBoolean() || manipManualCoral.getAsBoolean()), coralSubsystem);

  private final Command m_coralIn = Commands.runOnce(coralSubsystem::coralIn, coralSubsystem);
  private final Command m_coralOut = Commands.runOnce(coralSubsystem::coralOut, coralSubsystem);
  private final Command m_coralStop = Commands.runOnce(coralSubsystem::coralStop, coralSubsystem);
  private final Command m_zeroCoral = Commands.runOnce(coralSubsystem::zeroCoral, coralSubsystem);
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

  private final Command m_eleStop = Commands.runOnce(elevatorSubsystem::eleStop, elevatorSubsystem);
  private final Command m_eleReset = Commands.runOnce(elevatorSubsystem::eleReset, elevatorSubsystem);
  private final Command m_eleLift = Commands.runOnce(elevatorSubsystem::eleLift, elevatorSubsystem);
  private final Command m_eleDown = Commands.runOnce(elevatorSubsystem::eleDown, elevatorSubsystem);
  private final Command m_zeroPID = Commands.runOnce(elevatorSubsystem::zeroPID, elevatorSubsystem);

  private final Command m_elePID = Commands.run(() ->elevatorSubsystem.setIsManual(false), elevatorSubsystem);
  private final Command m_coralPID = Commands.run(() ->coralSubsystem.setCManual(false), coralSubsystem);

  /*******Set up drive********/
  public final CommandSwerveDrivetrain drivetrain = TunerConstants.DriveTrain; 
  private final LegacySwerveRequest.FieldCentric drive = new LegacySwerveRequest.FieldCentric()
      .withDriveRequestType(DriveRequestType.OpenLoopVoltage); 

  private final LegacySwerveRequest.SwerveDriveBrake brake = new LegacySwerveRequest.SwerveDriveBrake();
  private final LegacySwerveRequest.PointWheelsAt point = new LegacySwerveRequest.PointWheelsAt();
  private final Telemetry logger = new Telemetry(Constants.MaxSpeed);

  /******initialize additional subsystems*******/
  private final LimeLightSubsystem limelight =  new LimeLightSubsystem(drivetrain);

  /*********Autos************/
  public final Command testSwerve = new TestAuto(drivetrain, drive);

  /********Commands not included with Swerve Builder******/
  private LimeLightTest lime = new LimeLightTest(drivetrain, () -> joystick.getRawAxis(1), () -> joystick.getRawAxis(0), () -> joystick.getRawAxis(4));
  private LimeLightStare stare = new LimeLightStare(drivetrain, () -> joystick.getRawAxis(1), () -> joystick.getRawAxis(0), () -> joystick.getRawAxis(4));


  private void configureBindings() {

    drivetrain.setDefaultCommand(
      drivetrain.applyRequest(() -> {
          
          double smoothedVoltage = voltageFilter.calculate(RobotController.getBatteryVoltage());
          boolean isBatteryLow = smoothedVoltage < 9.5;
          
          SmartDashboard.putBoolean("CRITICAL LOW BATTERY", isBatteryLow);
          SmartDashboard.putNumber("Filtered Voltage", smoothedVoltage);
          joystick.getHID().setRumble(RumbleType.kBothRumble, isBatteryLow ? 0.5 : 0.0);

          double safeLimit = MathUtil.clamp((smoothedVoltage - 9.0) / (11.5 - 9.0) * (3.0 - 1.0) + 1.0, 1.0, 3.0);

          boolean isCreepMode = joystick.rightBumper().getAsBoolean();
          double activeLimit = isCreepMode ? 10.0 : safeLimit;
          double currentSpeedMultiplier = isCreepMode ? turtleSpeed : driveSpeed;

          double rawX = MathUtil.applyDeadband(-joystick.getRawAxis(1), 0.1);
          double rawY = MathUtil.applyDeadband(-joystick.getRawAxis(0), 0.1);
          double rawRot = MathUtil.applyDeadband(-joystick.getRawAxis(4), 0.1);

          double curveX = Math.copySign(rawX * rawX, rawX);
          double curveY = Math.copySign(rawY * rawY, rawY);
          double curveRot = Math.copySign(rawRot * rawRot, rawRot);

          double maxChangePerLoop = activeLimit * 0.02;

          currentX += MathUtil.clamp(curveX - currentX, -maxChangePerLoop, maxChangePerLoop);
          currentY += MathUtil.clamp(curveY - currentY, -maxChangePerLoop, maxChangePerLoop);
          currentRot += MathUtil.clamp(curveRot - currentRot, -maxChangePerLoop, maxChangePerLoop);

          return drive
              .withVelocityX(currentX * Constants.MaxSpeed * currentSpeedMultiplier)
              .withVelocityY(currentY * Constants.MaxSpeed * currentSpeedMultiplier)
              .withRotationalRate(currentRot * Constants.MaxAngularRate * currentSpeedMultiplier);
      })
    );
    joystick.y().whileTrue(drivetrain.applyRequest(() -> point.withModuleDirection(new Rotation2d(-joystick.getLeftY(), -joystick.getLeftX()))));
    joystick.y().onTrue(drivetrain.runOnce(() -> drivetrain.seedFieldRelative()));
    joystick.x().onTrue(lime);
    joystick.a().whileTrue(stare);
    joystick.b().onTrue(m_zeroPID);
    joystick.b().onTrue(m_zeroCoral);

    /*****Assigning Default Commands*****/
    algaeSubsystem.setDefaultCommand(m_algaeDriver);
    elevatorSubsystem.setDefaultCommand(m_eleDriver);
    coralSubsystem.setDefaultCommand(m_angleDriver);

    /*****Assigning Buttons*****/
    EleLeft.onTrue(m_eleReset);
    NoUp.onTrue(m_angleReset);

    /***Manip Flight Stick***/
    manipCoralIn.whileTrue(m_coralIn).onFalse(m_coralStop);
    manipCoralOut.whileTrue(m_coralOut).onFalse(m_coralStop);
    manipAlgaeIn.whileTrue(m_startIntake).onFalse(m_stopIntake);
    manipAlgaeOut.whileTrue(m_reverseIntake).onFalse(m_stopIntake);
    manipManualEle.whileFalse(m_elePID);
    manipManualCoral.whileFalse(m_coralPID);

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

    NamedCommands.registerCommand("Zero 1", m_zeroCoral);
    NamedCommands.registerCommand("Zero 2", m_zeroPID);

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

    configureBindings();
  }

  public Command getAutonomousCommand() {
    return pathChooser.getSelected();
  }
}
