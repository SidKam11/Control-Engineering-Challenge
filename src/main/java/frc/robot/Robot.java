// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.ctre.phoenix.motorcontrol.TalonFXControlMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import com.ctre.phoenix.motorcontrol.can.TalonFXConfiguration;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;


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
  private static final int ktarget = 10;
  private static final double kP = 0.00005;
  private static final double kI = 0.0001;
  private static final double kD = 0.00001;

  final int kUnitsPerRevolution = 2048;
  double errorSum = 0;
  double prevError = ktarget * kUnitsPerRevolution;

  // Initialize objects 
  private XboxController joystick = new XboxController(kJoystickPort); // Joystick
  private final WPI_TalonFX talon = new WPI_TalonFX(10); // TalonFX
  private final AnalogInput ultrasonic = new AnalogInput(kUltrasonicPort); // Ultrasonic sensor

  /**
   * This function is run when the robot is first started up and should be used for any
   * initialization code.
   */
  @Override
  public void robotInit() {

    
    // Reset TalonFX & perform any needed clean up

		/* newer config API */
		TalonFXConfiguration configs = new TalonFXConfiguration();
		/* select integ-sensor for PID0 (it doesn't matter if PID is actually used) */
		configs.primaryPID.selectedFeedbackSensor = FeedbackDevice.IntegratedSensor;
		/* config all the settings */
    talon.configAllSettings(configs);
    talon.setSelectedSensorPosition(0);
  }

  @Override
  public void teleopPeriodic() {
    double selSenPos = talon.getSelectedSensorPosition(0); /* position units */
    double selSenVel = talon.getSelectedSensorVelocity(0); /* position units per 100ms */

    double pos_Rotations = (double) selSenPos / kUnitsPerRevolution;
		double vel_RotPerSec = (double) selSenVel / kUnitsPerRevolution * 10; /* scale per100ms to perSecond */
    double vel_RotPerMin = vel_RotPerSec * 60.0;
    
    double error = ktarget * kUnitsPerRevolution - selSenPos;
    double deltaError = (error - prevError) / 0.02;
    double PIDmultiplier = (errorSum * kI) + (error * kP) + (deltaError * kD);
    
    prevError = error;

    // Set speed of motors (value between [-1,1]) by
    // taking input from joystick (value also between [-1, 1])
    // and scaling it down (so things don't break)
    // and using that value as the input
    // Get value from ultrasonic (12-bit value) and
    // convert to inches
    double distance = ultrasonic.getValue() * 0.0492;
    
    if(error / kUnitsPerRevolution < 1)
    {
      errorSum += error * 0.02;
    }


    if(PIDmultiplier > 0.1)
    {
      talon.set(TalonFXControlMode.PercentOutput, 0.1);
    } else {
      talon.set(TalonFXControlMode.PercentOutput, PIDmultiplier);
    }

    // Display distance on Smart Dashboard & print
    SmartDashboard.putNumber("Distance", distance);
    SmartDashboard.putNumber("Pos-units: ", selSenPos);
    SmartDashboard.putNumber("Vel-unitsPer100ms: ", selSenVel);
    SmartDashboard.putNumber("Pos-Rotations: ", pos_Rotations);
    SmartDashboard.putNumber("Vel-RPM: ", vel_RotPerMin);
    SmartDashboard.putNumber("Vel-RPS: ", vel_RotPerSec);
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