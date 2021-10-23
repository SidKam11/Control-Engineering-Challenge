// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.ctre.phoenix.motorcontrol.TalonFXControlMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;

import edu.wpi.first.wpilibj.AnalogInput;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

/**
 * The VM is configured to automatically run this class, and to call the functions corresponding to
 * each mode, as described in the TimedRobot documentation. If you change the name of this class or
 * the package after creating this project, you must also update the build.gradle file in the
 * project.
 */
public class Robot extends TimedRobot {

  // Create constants for ports to be used in the code
  private static final int kMotorPort = 7;
  private static final int kJoystickPort = 0;
  private static final int kUltrasonicPort = 0;

  // Initialize objects 
  private XboxController joystick = new XboxController(kJoystickPort); // Joystick
  private final WPI_TalonFX falcon = new WPI_TalonFX(10); // TalonFX
  private final AnalogInput ultrasonic = new AnalogInput(kUltrasonicPort); // Ultrasonic sensor

  /**
   * This function is run when the robot is first started up and should be used for any
   * initialization code.
   */
  @Override
  public void robotInit() {
    
    // Reset TalonFX & perform any needed clean up
    falcon.configFactoryDefault();

  }

  @Override
  public void teleopPeriodic() {

    // Set speed of motors (value between [-1,1]) by
    // taking input from joystick (value also between [-1, 1])
    // and scaling it down (so things don't break)
    // and using that value as the input
    falcon.set(TalonFXControlMode.PercentOutput, joystick.getRawAxis(1) * 0.1);

    // Get value from ultrasonic (12-bit value) and
    // convert to inches
    double distance = ultrasonic.getValue() * 0.0492;

    // Display distance on Smart Dashboard & print
    SmartDashboard.putNumber("Distance", distance);
    // System.out.println("Distance: " + distance);
  }


  // Overriden methods we don't care about atm

  @Override
  public void robotPeriodic() {}

  @Override
  public void autonomousInit() {}

  @Override
  public void autonomousPeriodic() {}

  @Override
  public void teleopInit() {}

  @Override
  public void disabledInit() {}

  @Override
  public void disabledPeriodic() {}

  @Override
  public void testInit() {}

  @Override
  public void testPeriodic() {}
}