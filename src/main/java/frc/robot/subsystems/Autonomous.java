// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.math.controller.HolonomicDriveController;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
import edu.wpi.first.math.trajectory.constraint.SwerveDriveKinematicsConstraint;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.SwerveControllerCommand;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.List;
import java.util.Scanner;

import javax.swing.plaf.synth.SynthScrollBarUI;

import com.ctre.phoenix.motorcontrol.can.VictorSPX;

import java.io.*;

public class Autonomous extends SubsystemBase {
  private Tracking Track;
  private SwerveDrive Swerve;
  public String AutoFile;
  private File TrajFile;
  private TrajectoryConfig TrajConfig;
  private Trajectory Trajectory;
  private SwerveDriveKinematicsConstraint SwerveDriveMaxSpeed;
  private Constraints PIDConstraints;
  private Scanner AutoReader;
  private List<String> Lines;
  private List<String> CurrentLine;
  private List<String> FileOrder;
  public List<String> AutoOrder;
  private List<Translation2d> Translation2ds;
  private List<Rotation2d> Rotation2ds;
  private List<Pose2d> Pose2ds;
  private List<Translation2d> MiddlePoints;
  public List<SwerveControllerCommand> SwerveControllerCommands;
  public Integer AutoStage;
  private Integer StartIndex;
  public Integer SwerveControllerCommandIndex;
  public Boolean IsScheduled = false;
  private double MaxSwerveVel;
  private double MaxSwerveAccel;

    // private VictorSPX armMotor = new VictorSPX(9); // 0 is the RIO PWM port this is connected to

    // // Claw open and close.
    // public DoubleSolenoid doubleSolenoidOne = new DoubleSolenoid(PneumaticsModuleType.CTREPCM, 0, 1);

    // // Level 1 Solenoid
    // public DoubleSolenoid doubleSolenoidTwo = new DoubleSolenoid(PneumaticsModuleType.CTREPCM, 2, 3);
  
    // // Level 2 Solenoid
    // public DoubleSolenoid doubleSolenoidThree = new DoubleSolenoid(PneumaticsModuleType.CTREPCM, 4, 5);
  

  public Autonomous(SwerveDrive SwerveDrive, Tracking Tracking) {
    Swerve = SwerveDrive;
    Track = Tracking;
    Lines = new ArrayList<String>();
    CurrentLine = new ArrayList<String>();
    FileOrder = new ArrayList<String>();
    AutoOrder = new ArrayList<String>();
    Translation2ds = new ArrayList<Translation2d>();
    Rotation2ds = new ArrayList<Rotation2d>();
    Pose2ds = new ArrayList<Pose2d>();
    MiddlePoints = new ArrayList<Translation2d>();
    SwerveControllerCommands = new ArrayList<SwerveControllerCommand>();
    AutoStage = 0;
    SwerveControllerCommandIndex = 0;
    MaxSwerveVel = 1;
    MaxSwerveAccel = 2;
    
  }

  public void initTrajectory() throws FileNotFoundException {
    TrajFile = Filesystem.getDeployDirectory().toPath().resolve("output/paths/" + AutoFile).toFile();
    AutoReader = new Scanner(TrajFile);
    SwerveDriveMaxSpeed = new SwerveDriveKinematicsConstraint(Swerve.Kinematics, MaxSwerveVel);
    PIDConstraints = new Constraints(MaxSwerveVel, MaxSwerveAccel);
    TrajConfig = new TrajectoryConfig(MaxSwerveVel, MaxSwerveAccel).setKinematics(Swerve.Kinematics).addConstraint(SwerveDriveMaxSpeed);
    Lines.clear();
    CurrentLine.clear();
    Translation2ds.clear();
    Rotation2ds.clear();
    Pose2ds.clear();
    SwerveControllerCommands.clear();
    FileOrder.clear();
    AutoOrder.clear();
    AutoStage = 0;
    Swerve.Gyro.reset();

    // Skip title line
    if (AutoReader.hasNextLine()) {
      AutoReader.nextLine();
    }
    // Parse entire auto file, and place each seperate line into an entry in the Lines list
    while (AutoReader.hasNextLine()) {
      Lines.add(AutoReader.nextLine());
    }
    AutoReader.close();
    // Pull all data entry-by-entry from the Lines list and add that data to new lists
    for (Integer Index = 0; Index <= Lines.size() - 1; Index++) {
      // Splits the next entry at each column, and adds that data to the CurrentLine list
      CurrentLine.addAll(Arrays.asList(Lines.get(Index).split(",")));
      // Skips adding the first "Move" string to the FileOrder list
      // This normalizes every series of "Move" entries to be 1 "Move" short of the actual number of points
      // This also helps because any commands on the first point will need to be done before the first move
      // So when we add the command here, it is before any "Move" entries
      if (Index == 0) {
        if (CurrentLine.size() == 7) {
          FileOrder.add(CurrentLine.get(6));
          addPointToLists();
        }
      }
      else {
        FileOrder.add("Move");
        // If there is a command at the point, add the command to the FileOrder list
        // Also adds a copy of the current point to the lists
        // This simplifies later code by having points for both the endpoint of this move and the beginning of the next
        if (CurrentLine.size() == 7) {
          FileOrder.add(CurrentLine.get(6));
          addPointToLists();
        }
      }
      
      addPointToLists();
      CurrentLine.clear();
    }
    // Set the position of the odometry to the starting position of the auto
    Swerve.Odometry.resetPosition(Swerve.GyroRotation2d, new SwerveModulePosition[] {Swerve.FrontRight.getPosition(), Swerve.FrontLeft.getPosition(), Swerve.BackLeft.getPosition(), Swerve.BackRight.getPosition()}, Pose2ds.get(0));

    System.out.println(FileOrder);

    // Create all required SwerveControllerCommands, as well as a roadmap for what to do at each step of auto
    for (Integer Index = 0; Index <= FileOrder.size() - 1; Index++) {
      // If the next command is to move, create a SwerveControllerCommand for every point up to the next non-move command
      if (FileOrder.get(Index) == "Move") {
        // Store the starting index, since this is the beginning point of the move, then increment the index
        StartIndex = Index++;
        // Create the list of midpoints
        System.out.println(Index);
        System.out.println(FileOrder.size() - 2);
        if (Index <= FileOrder.size() - 2) {
          while (Index <= FileOrder.size() - 2 & FileOrder.get(Index) == "Move") {
            MiddlePoints.add(Translation2ds.get(Index++));
          }
        }
        // Generate the trajectory, using the StartIndex for the starting position, the MiddlePoints list we just created, and the current index as the endpoint
        Trajectory = TrajectoryGenerator.generateTrajectory(Pose2ds.get(StartIndex), MiddlePoints, Pose2ds.get(Index), TrajConfig);
        // Generate the SwerveControllerCommand, and put it in the SwerveControllerCommandslist
        SwerveControllerCommands.add(new SwerveControllerCommand(Trajectory, Swerve::getPose, Swerve.Kinematics, new HolonomicDriveController(new PIDController(1, 0, 0), new PIDController(1, 0, 0), new ProfiledPIDController(1, 0, 0, PIDConstraints)), Swerve::setModuleStates, Swerve));
        // Decrement the index in preparation for the for loop to increment it
        Index--;
        MiddlePoints.clear();
      }
      // Add the command to the AutoOrder list, which will act as a roadmap for Auto
      AutoOrder.add(FileOrder.get(Index));
    }
    System.out.println("Start X:" + Translation2ds.get(0).getX());
    System.out.println("Start Y:" + Translation2ds.get(0).getY());
  }

  private void addPointToLists() {
    // Add the Translation2d of the point to the list
    Translation2ds.add(new Translation2d(Double.parseDouble(CurrentLine.get(0)),Double.parseDouble(CurrentLine.get(1))));
    // Check what alliance the auto is for, since Pathweaver doesn't take into account the alliance the robot is on
    if (AutoFile.contains("Red")) {
      // Add the Rotation2d of the point to the list, and invert it to solve the previously stated issue
      Rotation2ds.add(new Rotation2d(Math.atan2(Double.parseDouble(CurrentLine.get(3)), Double.parseDouble(CurrentLine.get(2))) + Math.PI));
    }
    if (AutoFile.contains("Blue")) {
      // Add the Rotation2d of the point to the list
      Rotation2ds.add(new Rotation2d(Math.atan2(Double.parseDouble(CurrentLine.get(3)), Double.parseDouble(CurrentLine.get(2)))));
    }
    // Also add the Pose2d of the point to the list, for the endpoints
    Pose2ds.add(new Pose2d(Translation2ds.get(Translation2ds.size() - 1), Rotation2ds.get(Rotation2ds.size() - 1)));
  }

  public void runAutonomous() {
    if (AutoStage <= AutoOrder.size() - 1) {
      if (AutoOrder.get(AutoStage) == "Move") {
        if (!IsScheduled) {
          System.out.println("Move");
          SwerveControllerCommands.get(SwerveControllerCommandIndex).andThen(() -> Swerve.stop()).schedule();
          IsScheduled = true;
        }
        if (SwerveControllerCommands.get(SwerveControllerCommandIndex).isFinished()) {
          AutoStage++;
          SwerveControllerCommandIndex++;
          IsScheduled = false;
        }
      }
    }
    if (AutoStage <= AutoOrder.size() - 1) {
      if (AutoOrder.get(AutoStage) == "Grab Cone") {
        System.out.println("Grab Cone");
        AutoStage++;
      }
    }
    if (AutoStage <= AutoOrder.size() - 1) {
      if (AutoOrder.get(AutoStage) == "BlueA1 Drop Cube") {
        System.out.println("Grab Cube");


        AutoStage++;
      }
    }
    if (AutoStage <= AutoOrder.size() - 1) {
      if (AutoOrder.get(AutoStage) == "Place Cone") {
        System.out.println("Place Cone");
        AutoStage++;
      }
    }
    if (AutoStage <= AutoOrder.size() - 1) {
      if (AutoOrder.get(AutoStage) == "Place Cube") {
        System.out.println("Place Cube");
        AutoStage++;
      }
    }
    if (AutoStage <= AutoOrder.size() - 1) {
      if (AutoOrder.get(AutoStage) == "Charge") {
        System.out.println("Charge");
        AutoStage++;
      }
    }
  }
}