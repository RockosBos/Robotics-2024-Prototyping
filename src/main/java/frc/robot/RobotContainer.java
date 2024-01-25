// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

//This project utilizes a modified version of FIRST Robotics Competition Team 364's Open source Swerve Drive code. 
//This project can be accessed from the following link: https://github.com/Team364/BaseFalconSwerve

package frc.robot;

import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.SampleSubsystem;
import frc.robot.subsystems.Swerve;
import frc.robot.commands.IntakeInFeed;
import frc.robot.commands.IntakeOutFeed;
import frc.robot.commands.IntakeStopFeed;
import frc.robot.commands.SetSampleHigh;
import frc.robot.commands.SetSampleLow;
import frc.robot.commands.SetSampleMotor;
import frc.robot.commands.stopSampleMotor;
import frc.robot.commands.Swerve.TeleopSwerve;

import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.Trigger;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and trigger mappings) should be declared here.
 */
public class RobotContainer {
  // The robot's subsystems and commands are defined here...
  
    /* Subsystems */
    //private final Joystick driver = new Joystick(0);
    private final XboxController driveController = new XboxController(0);
    private final XboxController operatorController = new XboxController(1);
    
    /* Subsystems */
    private final Swerve s_Swerve = new Swerve();
    private final SampleSubsystem s_SampleSubsystem = new SampleSubsystem();
    private final IntakeSubsystem s_IntakeSubsystem = new IntakeSubsystem();
    /* Drive Controls */
    private final int translationAxis = XboxController.Axis.kLeftY.value;
    private final int strafeAxis = XboxController.Axis.kLeftX.value;
    private final int rotationAxis = XboxController.Axis.kRightX.value;

    /* Driver Buttons */
    private final JoystickButton zeroGyro = new JoystickButton(driveController, XboxController.Button.kY.value);
    private final JoystickButton robotCentric = new JoystickButton(driveController, XboxController.Button.kLeftBumper.value);
    private final JoystickButton runSampleMotor = new JoystickButton(driveController, XboxController.Button.kA.value);
    private final JoystickButton IntakeInFeed = new JoystickButton(driveController, XboxController.Button.kB.value);
     private final JoystickButton IntakeOutFeed = new JoystickButton(driveController, XboxController.Button.kX.value);
  
    private final SendableChooser<Command> autonomousSelector = new SendableChooser<Command>();

    private SlewRateLimiter translationLimiter = new SlewRateLimiter(5);
    private SlewRateLimiter strafeLimiter = new SlewRateLimiter(5);
    private SlewRateLimiter rotationLimiter = new SlewRateLimiter(5);


    /** The container for the robot. Contains subsystems, OI devices, and commands. */
    public RobotContainer() {

        //SET DEFAULT COMMANDS  
        s_Swerve.setDefaultCommand(
            new TeleopSwerve(
                s_Swerve, 
                () -> translationLimiter.calculate(-driveController.getRawAxis(translationAxis)), 
                () -> strafeLimiter.calculate(-driveController.getRawAxis(strafeAxis)), 
                () -> rotationLimiter.calculate(-driveController.getRawAxis(rotationAxis)), 
                () -> robotCentric.getAsBoolean()
            )
        );

        s_SampleSubsystem.setDefaultCommand(new SetSampleLow(s_SampleSubsystem));
        s_IntakeSubsystem.setDefaultCommand(new IntakeStopFeed(s_IntakeSubsystem));

        CameraServer.startAutomaticCapture();

        // Configure the button bindings
        configureButtonBindings();

        putDashboard();
    }

    /**
     * Use this method to define your button->command mappings. Buttons can be created by
     * instantiating a {@link GenericHID} or one of its subclasses ({@link
     * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing it to a {@link
     * edu.wpi.first.wpilibj2.command.button.JoystickButton}.
     */
    private void configureButtonBindings() {
        /* Driver Buttons */
        zeroGyro.onTrue(new InstantCommand(() -> s_Swerve.zeroGyro()));
        runSampleMotor.whileTrue(new SetSampleHigh(s_SampleSubsystem));
        IntakeInFeed.whileTrue(new IntakeInFeed(s_IntakeSubsystem));
        IntakeOutFeed.whileTrue(new IntakeOutFeed(s_IntakeSubsystem));

    }
  public Command getAutonomousCommand() {
    // An example command will be run in autonomous
      return autonomousSelector.getSelected();
  }

  public void putDashboard(){
      SmartDashboard.putData("Autonomous Mode", autonomousSelector);
  }
}
