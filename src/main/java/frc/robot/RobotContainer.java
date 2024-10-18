// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;



import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.Swerve.Commands.SwerveTeleop;
import frc.robot.Swerve.Drive;



public class RobotContainer {
  
  Drive robotDrive; 
  CommandXboxController controller;

  public RobotContainer() {
    robotDrive = new Drive(); 
    robotDrive.setDefaultCommand(new SwerveTeleop(
                            () -> controller.getLeftX(), 
                            () -> controller.getLeftY(), 
                            () -> controller.getRightX(), 
                            robotDrive));
    configureBindings();
  }

  
  private void configureBindings() {
    
    
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    // An example command will be run in autonomous
    return null;
  }
}
