// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;

import com.revrobotics.CANSparkMax;
import com.revrobotics.SparkMaxRelativeEncoder;
import com.revrobotics.CANSparkMax.ControlType;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkMaxPIDController;

import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.NeutralMode;

// This class defines objects and variables for each wheel module
public class Wheel extends SubsystemBase {
  // Define objects and variables
  public CANSparkMax Drive;
  public RelativeEncoder DriveEncoder;
  public SparkMaxPIDController DrivePIDController;
  public TalonSRX Steer;
  public double SteerAngRot;
  public double SteerAngRad;
  public double SteerFullRot;
  public Translation2d Location;
  public SwerveModuleState ModuleState;
  public double DiffToAng;
  public double AngSpdMod;
  public double PrevRampedWheelSpd;
  public double RampedWheelSpd;
  public int EncoderIsNegative;
  public boolean IsInput;
  public SwerveModulePosition ModulePos;

  /**
	 * Class constructor for the Wheel class, initializes all variables, objects, and methods for the created Wheel object
	 *
	 * @param ModuleLocationX
	 *            X position of the wheel module relative to the center of the robot in meters
	 * @param ModuleLocationY
	 *            Y position of the wheel module relative to the center of the robot in meters
	 */
  public Wheel(double ModuleLocationX, double ModuleLocationY) {
    Location = new Translation2d(ModuleLocationX, ModuleLocationY);
    SteerAngRot = 0.0;
    SteerAngRad = 0.0;
    SteerFullRot = 0.0;
    DiffToAng = 0.0;
    AngSpdMod = 0.0;
    PrevRampedWheelSpd = 0.0;
    RampedWheelSpd = 0.0;
    IsInput = false;
  }

  /**
	 * Define what the objects "SteerEncoder" and "SteerPIDController" refer to, and initialize them
	 */
  public void initEncodersAndPIDControllers() {
    // Make the steer motors not move when there isn't an input
    Steer.setNeutralMode(NeutralMode.Brake);

    Steer.configClosedloopRamp(0);

    // Tell the Steer motor controller that an encoder exists, and what kind it is
    Steer.configSelectedFeedbackSensor(FeedbackDevice.Analog, 0, 0);

    // Define what encoder the object "DriveEncoder" refers to
    DriveEncoder = Drive.getEncoder(SparkMaxRelativeEncoder.Type.kHallSensor, 42);
    
    // Zero relative encoders, just in case
    DriveEncoder.setPosition(0);
    
    // Define what PID controller the object "DrivePIDController" refers to
    DrivePIDController = Drive.getPIDController();
    
    // Tell the PID controller what encoder to use
    DrivePIDController.setFeedbackDevice(DriveEncoder);

    // Set max and min values to be sent to the motors by the PID controllers. Likely shouldn't be changed.
    Steer.configClosedLoopPeakOutput(0, 1);
    DrivePIDController.setOutputRange(-1, 1);
  }

  /**
   * Set the P, I, and D values for the PID controllers
   * 
   * @param DFF
   *            Drive Feed-Forward value
   * @param DP
	 *            Drive Proportional value
   * @param DI
	 *            Drive Integral value
   * @param DD
	 *            Drive Derivative value
   * @param SP
	 *            Steer Proportional value
   * @param SI
	 *            Steer Integral value
   * @param SD
	 *            Steer Derivative value
   */
  public void setPIDValues(Double DFF, Double DP, Double DI, Double DD, Double SP, Double SI, Double SD) {
    DrivePIDController.setFF(DFF);
    DrivePIDController.setP(DP);
    DrivePIDController.setI(DI);
    DrivePIDController.setD(DD);
    Steer.config_kP(0, SP);
    Steer.config_kI(0, SI);
    Steer.config_kD(0, SD);
  }

  /**
   * Do math and set multiple variables required to make the absolute encoders function properly
   * 
   * @param EncoderPosMod
	 *            The value output by the encoders when at one full rotation
   */
  public void setEncoderVariables(double EncoderPosMod) {
    // Convert the raw encoder output into a unit of rotations
    SteerAngRot = (Steer.getSelectedSensorPosition() / EncoderPosMod);

    // Get the number of full rotations the wheel has gone through
    SteerFullRot = Math.floor(SteerAngRot);

    // Invert the angle, so wpilib's math has a good input
    SteerAngRad = ((2 * Math.PI) - ((2 * Math.PI) * (SteerAngRot - SteerFullRot)));
  }

  /**
   * Do math for the swerve drive that each wheel has to call, and then output the desired angle and speed to the wheel
   * 
   * @param X
	 *            Desired X speed of the robot from -1 to 1
   * @param Y
	 *            Desired Y speed of the robot from -1 to 1
   * @param Spin
	 *            Desired rotational speed of the robot from -1 to 1
   * @param EncoderPosMod
	 *            Number to modify the encoders' output values, in order to get the position to a common scale of 1, so math can be done
   * @param DriveRampValue
	 *            Amount the drive speed can increase or decrease by, max value of 2, min value of 0
   */
  public void optimizeAndCalculateVariables(double DriveRampValue) {
    // Optimize rotation positions, so the wheels don't turn 180 degrees rather than just spinning the drive motor backwards
    // Determine if the distance between the desired angle and the current angle is less than or equal to 90
    // This is to determine whether the drive motors should be driven forward or backward.
    // If the difference between the desired and current positions is less than or equal to 90 degrees, then...
    if ((Math.abs(ModuleState.angle.getRadians() - SteerAngRad) <= (Math.PI / 2)) || (Math.abs(ModuleState.angle.getRadians() - SteerAngRad) >= ((3 * Math.PI) / 2))) {
      // If the wheel would have to cross into a new rotation to travel the shortest distance to the desired angle, then...
      if (Math.abs(ModuleState.angle.getRadians() - SteerAngRad) >= ((3 * Math.PI) / 2)) {
        // If the difference is positive, then...
        if (ModuleState.angle.getRadians() > SteerAngRad) {
          // Subtract 2pi from the desired angle to show the PID controller later that the shortest distance is to cross 0
          ModuleState = new SwerveModuleState(ModuleState.speedMetersPerSecond, new Rotation2d(ModuleState.angle.getRadians() - (2 * Math.PI)));
        }
        // If the difference is negative, then...
        else {
          // Add 2pi to the desired angle to show the PID controller later that the shortest distance is to cross 2pi
          ModuleState = new SwerveModuleState(ModuleState.speedMetersPerSecond, new Rotation2d(ModuleState.angle.getRadians() + (2 * Math.PI)));
        }
      }

    }
    // If the difference between the desired and current positions is greater than 90 degrees, then...
    else {
      // If the difference is positive, then...
      if (ModuleState.angle.getRadians() > SteerAngRad) {
        // Invert the Drive motor output, and flip the desired angle by subtracting pi
        ModuleState = new SwerveModuleState(-ModuleState.speedMetersPerSecond, new Rotation2d(ModuleState.angle.getRadians() - Math.PI));
      }
      // If the difference is negative, then...
      else {
        // Invert the Drive motor output, and flip the desired angle by adding pi
        ModuleState = new SwerveModuleState(-ModuleState.speedMetersPerSecond, new Rotation2d(ModuleState.angle.getRadians() + Math.PI));
      }
    }

    // Re-invert the angle, so the PID controller has a good input
    ModuleState = new SwerveModuleState(ModuleState.speedMetersPerSecond, new Rotation2d(((2 * Math.PI) - ModuleState.angle.getRadians()) + (SteerFullRot * (2 * Math.PI))));

    // Check if any input is being sent, to prevent wheels from rotating to 0 when no input. 
    if (ModuleState.speedMetersPerSecond != 0) {
      IsInput = true;
    }
    else {
      IsInput = false;
    }
  }

  public void setOutputs(double EncoderPosMod) {
    // Tell the steer motor to turn the wheel to the correct position
    // An issue is created by ramping which this if statement solves, I will explain the root of the problem, and the solution here:
    // If all inputs for robot speeds are 0, the angle for the wheel will default to 0
    // This causes a problem because the drive wheel speed does not instantly go to zero, causing the robot's direction to change
    // This if statement fixes this issue by only changing the angle of the wheel if and only if any of the desired robot speeds are greater than 0
    if (IsInput == true) {
      Steer.set(ControlMode.Position, ((ModuleState.angle.getDegrees() / 360.0) * EncoderPosMod));
    }

    //Steer.set(ControlMode.Position, 0);

    // Tell the drive motor to drive the wheels at the correct speed
    DrivePIDController.setReference((((ModuleState.speedMetersPerSecond / ((4 / 39.37) * Math.PI)) * 60) / .15), ControlType.kVelocity);
  }

  public SwerveModulePosition getPosition() {
    ModulePos = new SwerveModulePosition((((DriveEncoder.getPosition()) * .15) * ((4 / 39.37) * Math.PI)), new Rotation2d(SteerAngRad).unaryMinus());
    return ModulePos;
  }
}