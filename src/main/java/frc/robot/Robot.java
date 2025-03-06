// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import choreo.auto.AutoChooser;
import choreo.auto.AutoFactory;
import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.cscore.UsbCamera;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.subsystems.DriveSubsystem;
// import frc.robot.subsystems.Drivetrain; // Commented out as the class does not exist
import edu.wpi.first.wpilibj2.command.button.RobotModeTriggers;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.cscore.CvSink;
import edu.wpi.first.cscore.CvSource;
import edu.wpi.first.cscore.UsbCamera;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.XboxController.Button;
import edu.wpi.first.wpilibj.simulation.XboxControllerSim;

import java.util.List;

import org.opencv.core.Mat;
import org.opencv.core.Point;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;
import org.photonvision.PhotonCamera;
import org.photonvision.targeting.PhotonTrackedTarget;
public class Robot extends TimedRobot {

  private static final double VISION_TURN_kP = 0.1; // Adjust this value as needed

  Thread m_visionThread;
    
  
  private Command m_autonomousCommand;

 
 
  private AutoChooser autoChooser;
  private RobotContainer m_robotContainer;
  private int loopCounter = 0;

  private PhotonCamera camera;

  public Robot() {
  // Change this to match the name of your camera
    camera = new PhotonCamera("photonvision");
    // Query the latest result from PhotonVision
    var result = camera.getLatestResult();
    //Check for targets
    

}
  
  
/**
   * This function is run when the robot is first started up and should be used for any
   * initialization code.
   */
  @Override
  public void robotInit() {
    // Instantiate our RobotContainer.  This will perform all our button bindings, and put our
    // autonomous chooser on the dashboard.
    m_robotContainer = new RobotContainer();

        autoChooser = new AutoChooser();
        

        autoChooser = new AutoChooser();

        // Add options to the chooser
        //      autoChooser.addRoutine("Forward 180",m_robotContainer.Forward() );
        autoChooser.addRoutine("Forward Right", m_robotContainer::forwardRightRoutine);
        autoChooser.addRoutine("Forward 180", m_robotContainer::forward180Routine);
        autoChooser.addRoutine("MAlgae to RSource", m_robotContainer::MAlgaetoRSource);
        autoChooser.addRoutine("RSource to Reef4", m_robotContainer::RSourcetoReef4Routine);
        autoChooser.addRoutine("ReefFullLoop", m_robotContainer::ReefFullLoopRoutine);
       // autoChooser.addRoutine("forwardrighttop", m_robotContainer::forwardrighttop);



        // Put the auto chooser on the dashboard
        SmartDashboard.putData("Auto Chooser", autoChooser);

        // Schedule the selected auto during the autonomous period
        RobotModeTriggers.autonomous().whileTrue(autoChooser.selectedCommandScheduler());

    
  }

  /**
   * This function is called every 20 ms, no matter the mode. Use this for items like diagnostics
   * that you want ran during disabled, autonomous, teleoperated and test.
   *
   * <p>This runs after the mode specific periodic functions, but before LiveWindow and
   * SmartDashboard integrated updating.
   */
  @Override
  public void robotPeriodic() {
    // Runs the Scheduler.  This is responsible for polling buttons, adding newly-scheduled
    // commands, running already-scheduled commands, removing finished or interrupted commands,
    // and running subsystem periodic() methods.  This must be called from the robot's periodic
    // block in order for anything in the Command-based framework to work.
    CommandScheduler.getInstance().run();
    if(loopCounter > 25) {
      System.out.println("Pose X Meters" + m_robotContainer.getDrive().getPose().getX());
      System.out.println("Pose Y Meters" + m_robotContainer.getDrive().getPose().getY());
      loopCounter = 0;
    } else {
      loopCounter++;
    }
  }

  /** This function is called once each time the robot enters Disabled mode. */
  @Override
  public void disabledInit() {}

  @Override
  public void disabledPeriodic() {}

  /** This autonomous runs the autonomous command selected by your {@link RobotContainer} class. */
  @Override
  public void autonomousInit() {
      //m_autonomousCommand = m_robotContainer.getAutonomousCommand();
      //m_autonomousCommand = m_robotContainer.moveLCommand();

    /*
     * String autoSelected = SmartDashboard.getString("Auto Selector",
     * "Default"); switch(autoSelected) { case "My Auto": autonomousCommand
     * = new MyAutoCommand(); break; case "Default Auto": default:
     * autonomousCommand = new ExampleCommand(); break; }
     */

    // schedule the autonomous command (example)
    /*  if (m_autonomousCommand != null) {
      m_autonomousCommand.schedule();
    }   
      */
  
  }

  /** This function is called periodically during autonomous. */
  @Override
  public void autonomousPeriodic() {}

  @Override
  public void teleopInit() {
    // This makes sure that the autonomous stops running when
    // teleop starts running. If you want the autonomous to
    // continue until interrupted by another command, remove
    // this line or comment it out.
    if (m_autonomousCommand != null) {
      m_autonomousCommand.cancel();
    }
  }

  /** This function is called periodically during operator control. */
  @Override
  public void teleopPeriodic() {
    
 final class Constants {

  // Existing constants



   static final class Swerve {

      public static final double kMaxLinearSpeed = 3.0; // Example value, adjust as needed

      public static final double kMaxAngularSpeed = 2.0; // Example value, adjust as needed

  }

}

   // Calculate drivetrain commands from Joystick values
   XboxController xboxController = new XboxController(0);
   double forward = -xboxController.getLeftY() * Constants.Swerve.kMaxLinearSpeed;
   double strafe = -xboxController.getLeftX() * Constants.Swerve.kMaxLinearSpeed;
   double turn = -xboxController.getRightX() * Constants.Swerve.kMaxAngularSpeed;

   // Read in relevant data from the Camera
   boolean targetVisible = false;
   double targetYaw = 0.0;
   var results = camera.getAllUnreadResults();
   if (!results.isEmpty()) {
       // Camera processed a new frame since last
       // Get the last one in the list.
       var result = results.get(results.size() - 1);
       if (result.hasTargets()) {
           // At least one AprilTag was seen by the camera
           for (var target : result.getTargets()) {
               if (target.getFiducialId() == 7) {
                   // Found Tag 7, record its information
                   targetYaw = target.getYaw();
                   targetVisible = true;
               }
           }
       }
   }

   // Auto-align when requested
   if (xboxController.getAButton() && targetVisible) {
       // Driver wants auto-alignment to tag 7
       // And, tag 7 is in sight, so we can turn toward it.
       // Override the driver's turn command with an automatic one that turns toward the tag.
       turn = -1.0 * targetYaw * VISION_TURN_kP * Constants.Swerve.kMaxAngularSpeed;
   }

   // Command drivetrain motors based on target speeds
   m_robotContainer.getDrive().drive(forward, strafe, turn,false,true);

   // Put debug information to the dashboard
   SmartDashboard.putBoolean("Vision Target Visible", targetVisible);
  }

  @Override
  public void testInit() {
    // Cancels all running commands at the start of test mode.
    CommandScheduler.getInstance().cancelAll();
  }

  /** This function is called periodically during test mode. */
  @Override
  public void testPeriodic() {}



  

    
}
  

