// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;


import edu.wpi.first.wpilibj.Compressor;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.DoubleSolenoid.*;
import edu.wpi.first.wpilibj.shuffleboard.*;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandScheduler;

import java.io.FileNotFoundException;

import javax.lang.model.util.ElementScanner14;

import com.fasterxml.jackson.databind.node.ArrayNode;
import com.revrobotics.CANSparkMax;
import com.revrobotics.SparkMaxPIDController;
import com.revrobotics.CANSparkMax.ControlType;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.SparkMaxRelativeEncoder;
import edu.wpi.first.networktables.*;
import edu.wpi.first.util.sendable.SendableBuilder;
import frc.robot.subsystems.Autonomous;
import frc.robot.subsystems.SwerveDrive;
import frc.robot.subsystems.Tracking;



public class Robot extends TimedRobot {
  // Define objects and variables
  private Joystick LeftStick;
  private Joystick RightStick;

  private XboxController ControllerTwo;
  private XboxController ControllerOne;

  private double ControllerOneX;
  private double ControllerOneY;
  private double ControllerOneTwist;
  private double[] MotorCurrents;
  private String[] AutoNames;
  private String PrevAuto;

  public SwerveDrive SwerveDrive;
  public Tracking Tracking;
  public Autonomous Autonomous;

  public NetworkTableInstance Inst;
  public NetworkTable DriverStation;
  public NetworkTableEntry GyroAng;
  public SendableChooser<String> AutoChooser;
  private ShuffleboardTab LiveWindow;
  private SimpleWidget FFGain;
  private SimpleWidget PGain;
  private SimpleWidget IGain;
  private SimpleWidget DGain;

  private CANSparkMax ArmAngle;
  private SparkMaxPIDController ArmAnglePIDController;
  private CANSparkMax ArmExtend;
  private SparkMaxPIDController ArmExtendPIDController;

  private Compressor Pump;
  private DoubleSolenoid Grabber;


  @Override
  public void robotInit() {
    Inst = NetworkTableInstance.getDefault();

    AutoChooser = new SendableChooser<String>();
    AutoNames = Filesystem.getDeployDirectory().toPath().resolve("output/paths").toFile().list();
    for (Integer Index = 0; Index <= AutoNames.length - 1; Index++) {
      AutoChooser.addOption(AutoNames[Index], AutoNames[Index]);
    }
    AutoChooser.setDefaultOption("BlueTest", "BlueTest");
    SmartDashboard.putData("AutoChooser", AutoChooser);

    LiveWindow = Shuffleboard.getTab("LiveWindow");

    FFGain = LiveWindow.add("FFGain", 0.000175);
    PGain = LiveWindow.add("PGain", 0.000005);
    IGain = LiveWindow.add("IGain", 0.0000004);
    DGain = LiveWindow.add("DGain", 0.0);

    // Assign joysticks to the "LeftStick" and "RightStick" objects
    LeftStick = new Joystick(0);
    RightStick = new Joystick(1);

    ControllerOne = new XboxController(0);
    ControllerTwo = new XboxController(1);
    

    ArmAngle = new CANSparkMax(10, MotorType.kBrushless);
    ArmAnglePIDController = ArmAngle.getPIDController();
    ArmExtend = new CANSparkMax(11, MotorType.kBrushless);
    ArmExtendPIDController = ArmExtend.getPIDController();

    ArmAnglePIDController.setFeedbackDevice(ArmAngle.getEncoder(SparkMaxRelativeEncoder.Type.kHallSensor, 42));
    ArmAnglePIDController.setOutputRange(-1, 1);
    ArmAnglePIDController.setFF(0);
    ArmAnglePIDController.setP(.05);
    ArmAnglePIDController.setI(0);
    ArmAnglePIDController.setD(0);
    ArmExtendPIDController.setFeedbackDevice(ArmExtend.getEncoder(SparkMaxRelativeEncoder.Type.kHallSensor, 42));
    ArmExtendPIDController.setOutputRange(-1, 1);
    ArmAnglePIDController.setFF(0);
    ArmAnglePIDController.setP(.06);
    ArmAnglePIDController.setI(0);
    ArmAnglePIDController.setD(0);

    Pump = new Compressor(0, PneumaticsModuleType.CTREPCM);
    Grabber = new DoubleSolenoid(PneumaticsModuleType.CTREPCM, 2, 3);
    Grabber.set(Value.kForward);

    // Instantiate an object for each class
    SwerveDrive = new SwerveDrive();
    Tracking = new Tracking(SwerveDrive);
    Autonomous = new Autonomous(SwerveDrive, Tracking);

    SwerveDrive.GyroRotation2d = SwerveDrive.Gyro.getRotation2d();

    // Call SwerveDrive methods, their descriptions are in the SwerveDrive.java file
    SwerveDrive.initMotorControllers(1, 2, 7, 8, 5, 6, 3, 4);
    SwerveDrive.setPID(0.000175, 0.000005, 0.0000004, 0.0, 8.0, 0.01, 0.01);
    // SwerveDrive.setPID(0.000175, 0.000001, 0.0000004, 0.0, 8.0, 0.01, 0.01);
    SwerveDrive.initKinematicsAndOdometry();
    PrevAuto = AutoChooser.getSelected();
    Autonomous.AutoFile = AutoChooser.getSelected();
    if (AutoChooser.getSelected() != null) {
      System.out.println(Autonomous.AutoFile);
    try {
        Autonomous.initTrajectory();
      } catch (FileNotFoundException e) {
        System.out.println("AUTO NOT FOUND");
      }
    }
    Pump.enableDigital();
  }

  @Override
  public void robotPeriodic() {
    CommandScheduler.getInstance().run();
    if (PrevAuto != AutoChooser.getSelected()) {
      Autonomous.AutoFile = AutoChooser.getSelected();
      System.out.println(Autonomous.AutoFile);
      try {
        Autonomous.initTrajectory();
      } catch (FileNotFoundException e) {
        System.out.println("AUTO NOT FOUND");
      }
      PrevAuto = AutoChooser.getSelected();
    }
    SwerveDrive.setPID(FFGain.getEntry().getDouble(0), PGain.getEntry().getDouble(0), IGain.getEntry().getDouble(0), DGain.getEntry().getDouble(0), 8.0, 0.01, 0.01);
  }
 
  @Override
  public void teleopInit() {
  }

  @Override
  public void teleopPeriodic() {
    SwerveDrive.GyroRotation2d = SwerveDrive.Gyro.getRotation2d().unaryMinus();

    SwerveDrive.FrontLeft.DriveEncoder.getVelocity();

    // Assign stick inputs to variables, to prevent discrepancies
    ControllerOneX = ControllerOne.getLeftX();
    ControllerOneY = ControllerOne.getLeftY();
    ControllerOneTwist = ControllerOne.getRightX();

    // Create deadzones on the joysticks, to prevent stick drift
    if (Math.abs(ControllerOneX) < 0.03) {
      ControllerOneX = 0.0;
    }
    if (Math.abs(ControllerOneY) < 0.03) {
      ControllerOneY = 0.0;
    }
    if (Math.abs(ControllerOneTwist) < 0.03) {
      ControllerOneTwist = 0.0;
    }

    if (LeftStick.getRawButton(2)) {
      Tracking.centerOnPole();
    }
    else {
      // Call swerveDrive() method, to do all the math and outputs for swerve drive
      SwerveDrive.swerveDrive(Math.pow(ControllerOneX, 3) * 2, (Math.pow(ControllerOneY, 3) * -2), (Math.pow(ControllerOneTwist, 3) * 2.5), (1.00), (1.00));
      SwerveDrive.setVariablesAndOptimize();
      SwerveDrive.setSwerveOutputs();
    }

    System.out.println(SwerveDrive.BackRight.Steer.getSelectedSensorPosition());


    SmartDashboard.putNumber("Gyro", SwerveDrive.GyroRotation2d.getDegrees());

    // Controller One
    SmartDashboard.putNumber("Left Y Input", ControllerOne.getLeftY());
    SmartDashboard.putNumber("Right X Input", ControllerOne.getRightX());

    SmartDashboard.putNumber("FrontLeft Absolute Output", SwerveDrive.FrontLeft.Steer.getSelectedSensorPosition());
    SmartDashboard.putNumber("FrontRight Absolute Output", SwerveDrive.FrontRight.Steer.getSelectedSensorPosition());
    SmartDashboard.putNumber("BackLeft Absolute Output", SwerveDrive.BackLeft.Steer.getSelectedSensorPosition());
    SmartDashboard.putNumber("BackRight Absolute Output", SwerveDrive.BackRight.Steer.getSelectedSensorPosition());

    MotorCurrents = new double[] {SwerveDrive.FrontLeft.Drive.getOutputCurrent(), SwerveDrive.FrontRight.Drive.getOutputCurrent(), SwerveDrive.BackLeft.Drive.getOutputCurrent(), SwerveDrive.BackRight.Drive.getOutputCurrent()};
    SmartDashboard.putNumberArray("RobotDrive Motors", MotorCurrents);
  
    if (RightStick.getRawButtonPressed(2) == true) {
      SwerveDrive.Gyro.reset();
      ArmAngle.getEncoder().setPosition(0);
      ArmExtend.getEncoder().setPosition(0);
    }
    if (LeftStick.getRawButtonPressed(4)) {
      ArmAnglePIDController.setReference(-22, ControlType.kPosition);
    }
    else if (LeftStick.getRawButtonPressed(6)) {
      ArmAnglePIDController.setReference(0, ControlType.kPosition);
    }
    if (LeftStick.getRawButtonPressed(5)) {
      ArmExtendPIDController.setReference(0, ControlType.kPosition);
    }
    else if (LeftStick.getRawButtonPressed(3)) {
      ArmExtendPIDController.setReference(56, ControlType.kPosition);
    }
    if (LeftStick.getRawButtonPressed(1)) {
      Grabber.toggle();
    }
  }

  //Autonomous right away
  @Override
  public void autonomousInit(){
  }

  //Autonomous repeat
  @Override
  public void autonomousPeriodic(){
    SwerveDrive.GyroRotation2d = SwerveDrive.Gyro.getRotation2d(); 
    Autonomous.runAutonomous();
  }
}