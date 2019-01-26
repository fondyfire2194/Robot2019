/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Preferences;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.command.Command;
import edu.wpi.first.wpilibj.command.Scheduler;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.commands.PathfinderReverseTrajectory;
import frc.robot.commands.PathfinderTrajectory;
import frc.robot.commands.RobotOrient;
import frc.robot.commands.RobotDriveToTarget;

import frc.robot.commands.Auto.*;
import frc.robot.BuildTrajectory;
import frc.robot.TrajDict;
import frc.robot.subsystems.DriveTrain;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.RobotRotate;
import jaci.pathfinder.Trajectory;
import frc.robot.AutoChoosers;
import frc.robot.VisionData;
import frc.robot.AutoCommands;
import frc.robot.LimeLight;
import frc.robot.LimelightControlMode.*;
import frc.robot.LoadFiles;

/**
 * The VM is configured to automatically run this class, and to call the
 * functions corresponding to each mode, as described in the TimedRobot
 * documentation. If you change the name of this class or the package after
 * creating this project, you must also update the build.gradle file in the
 * project.
 */

public class Robot extends TimedRobot {
  public static DriveTrain driveTrain = null;
  public static RobotRotate robotRotate;
  public static Elevator elevator;
  public static SimpleCSVLogger simpleCSVLogger;
  public static LimeLight limelightCamera;
  public static VisionData visionData;
  public static AutoChoosers autoChoosers;
  public static String chosenFileName = "Dummy";
  public static OI m_oi;
  public static Preferences prefs;
  public static BuildTrajectory buildTrajectory;

  public static Command[] autonomousCommand;

  public static Command autoTimeDelayCommand;
  double autoTimeDelaySeconds;

  public static boolean[] autonomousCommandDone;

  boolean autoTimeDelayDone;

  SendableChooser<Command> m_chooser = new SendableChooser<>();
  public static boolean doRobotPosition;
  public static double positionTargetFt;
  public static double positionFPS;
  public static boolean incrPosn;
  public static boolean isPositioning;
  public static boolean stopPositioning;
  public static boolean orientRunning;
  public static boolean reverseOrient;
  public static boolean doTeleopOrient;
  public static boolean doTeleopPosition;

  public static double targetPosition;
  public static double targetRate;
  public static double xPosition;
  public static double yPosition;

  private double angleTarget = 90;
  private double orientRate = 0.5;

  public enum motionType {
    incremental, absolute
  }

  public static boolean useGainPrefs = true;
  public static Trajectory[] activeTrajectory;

  public static String activeTrajName = "Empty";
  public static Trajectory[] leftBufferTrajectory = new Trajectory[6];
  public static Trajectory[] rightBufferTrajectory = new Trajectory[6];
  public static int secondHatchIndex;
  public static String[] bufferTrajectoryName = { "0", "1", "2", "3", "4", "5" };

  public static String bufferTrajName = "Empty";
  public static String testTrajectoryName;
  public static int testTrajectoryDirection;

  public static double[] activeTrajectoryGains = { 0, 0, 0, 0 };
  public static double[] bufferTrajectoryGains = { 0, 0, 0, 0 };
  public static boolean doTeleopTrajectory;
  public static boolean revTraj;;
  public static boolean doFileTrajectory;
  public static boolean trajectoryRunning;
  public static boolean buildOK;

  public static String[] names = { "Step", "LeftCmd", "LeftFt", "RightCmd", "RightFt", "AngleCmd", "Angle",
      "LeftSegVel", "left", "ActLeftVel", "RightSegVel", "right", "ActRightVel", "turn" };
  public static String[] units = { "Number", "FT", "FT", "FT", "FT", "Deg", "Deg", "pct", "pct", "pct", "pct", "pct",
      "pct", "pct" };
  public static boolean createTrajectoryRunFile = true;
  public static double trajectoryX;
  public static double trajectoryY;
  public static double trajectoryAngle;
  public static String trajFileName;
  public static String logName;
  public static boolean robotMoveReverse;
  int test;
  public static boolean buildInProgress;
  public static int startPositionSelected;
  public static int secondHatchSelected;
  public static boolean useUsb = true;
  public static boolean faceField = true;
  public static boolean invertY = true;
  public static boolean trajectoriesLoaded;

  public static int numberOfAutonomousCommands;
  public static boolean startSettingsReady = false;
  public static boolean startSettingsDone = false;
  public static boolean readingRunning = false;

  LoadFiles currentLoader = new LoadFiles();
  LoadFiles oldLoader = null;
  boolean firstTime = false;
  public static String runningCommandName = "None";

  /**
   * This function is run when the robot is first started up and should be used
   * for any initialization code.
   */
  @Override
  public void robotInit() {
    driveTrain = new DriveTrain();
    robotRotate = new RobotRotate();
    m_oi = new OI();
    simpleCSVLogger = new SimpleCSVLogger();
    limelightCamera = new LimeLight();
    visionData = new VisionData();
    buildTrajectory = new BuildTrajectory();

    autoChoosers = new AutoChoosers();
    // autoChoosers.init();
    autonomousCommand = new Command[10];
    autonomousCommandDone = new boolean[10];
    prefs = Preferences.getInstance();
    // Pref.deleteAllPrefs();
    // Pref.deleteUnused();
    Pref.addMissing();
    SmartDashboard.putData(driveTrain);
    Timer.delay(.02);
    SmartDashboard.putNumber("Target Feet", 5);
    SmartDashboard.putNumber("Position FPS", 5);
    SmartDashboard.putNumber("Target Angle", angleTarget);
    Timer.delay(.02);
    SmartDashboard.putNumber("Orient Rate", orientRate);
    Timer.delay(.02);
    SmartDashboard.putBoolean("RevOrient", false);
    Timer.delay(.02);
    SmartDashboard.putBoolean("InvertY", false);
    Timer.delay(.02);
    SmartDashboard.putBoolean("UseGainPrefs", true);
    Timer.delay(.02);
    SmartDashboard.putBoolean("StartSettingsReady", false);
    SmartDashboard.putData(driveTrain);
    // SmartDashboard.putData(elevator);
    SmartDashboard.putData(robotRotate);

    SmartDashboard.putData(Scheduler.getInstance());
  }

  /**
   * This function is called every robot packet, no matter the mode. Use this for
   * items like diagnostics that you want ran during disabled, autonomous,
   * teleoperated and test.
   *
   * <p>
   * This runs after the mode specific periodic functions, but before LiveWindow
   * and SmartDashboard integrated updating.
   */
  @Override
  public void robotPeriodic() {
    updateStatus();
  }

  /**
   * This function is called once each time the robot enters Disabled mode. You
   * can use it to reset any subsystem information you want to clear when the
   * robot is disabled.
   */
  @Override
  public void disabledInit() {

  }

  @Override
  public void disabledPeriodic() {
    Scheduler.getInstance().run();
    startSettingsReady = SmartDashboard.getBoolean("StartSettingsReady", false);

    SmartDashboard.putBoolean("StartSettingsDone", startSettingsDone);
    if (startSettingsDone) {
      startSettingsDone = startSettingsReady;
    }
    if (startSettingsReady && !startSettingsDone && !readingRunning) {
      readTrajFiles();
      readingRunning = true;
    }
  }

  /**
   * This autonomous (along with the chooser code above) shows how to select
   * between different autonomous modes using the dashboard. The sendable chooser
   * code works with the Java SmartDashboard. If you prefer the LabVIEW Dashboard,
   * remove all of the chooser code and uncomment the getString code to get the
   * auto name from the text box below the Gyro
   *
   * <p>
   * You can add additional auto modes by adding additional commands to the
   * chooser code above (like the commented example) or additional comparisons to
   * the switch structure below with additional strings & commands.
   */
  @Override
  public void autonomousInit() {
    Scheduler.getInstance().run();
    Robot.runningCommandName = "None";
    autoTimeDelaySeconds = AutoChoosers.timeDelayChooser.getSelected();

    startPositionSelected = AutoChoosers.startPositionChooser.getSelected();
    if (startPositionSelected == 0)
      autoTimeDelaySeconds = 0;

    secondHatchSelected = AutoChoosers.secondHatchChooser.getSelected();

    autonomousCommand[0] = new AutoWait(autoTimeDelaySeconds);

    // schedule the autonomous command (example)
    if (autonomousCommand[0] != null)
      autonomousCommand[0].start();
  }

  /**
   * This function is called periodically during autonomous.
   */
  @Override
  public void autonomousPeriodic() {
    Scheduler.getInstance().run();

    if (startPositionSelected != 0) {

      switch (startPositionSelected) {
      case 1:
        numberOfAutonomousCommands = AutoCommands.setLeftStart();
      case 2:
        numberOfAutonomousCommands = AutoCommands.setLeftCenterStart();
      case 3:
        numberOfAutonomousCommands = AutoCommands.setRightCenterStart();
      case 4:
        numberOfAutonomousCommands = AutoCommands.setRightStart();

      }

      boolean rightStart = startPositionSelected >=3;

      numberOfAutonomousCommands = AutoCommands.secondHatchCommands(secondHatchSelected, numberOfAutonomousCommands, rightStart);

      if (autonomousCommandDone[0])
        autonomousCommand[1].start();
      if (autonomousCommandDone[1] && numberOfAutonomousCommands > 1)
        autonomousCommand[2].start();
      if (autonomousCommandDone[2] && numberOfAutonomousCommands > 2)
        autonomousCommand[3].start();
      if (autonomousCommandDone[3] && numberOfAutonomousCommands > 3)
        autonomousCommand[4].start();
      if (autonomousCommandDone[4] && numberOfAutonomousCommands > 4)
        autonomousCommand[5].start();
        if (autonomousCommandDone[5] && numberOfAutonomousCommands > 5)
        autonomousCommand[6].start();
      if (autonomousCommandDone[6] && numberOfAutonomousCommands > 6)
        autonomousCommand[7].start();
      if (autonomousCommandDone[7] && numberOfAutonomousCommands > 8)
        autonomousCommand[8].start();

    }
  }

  @Override
  public void teleopInit() {
    Robot.runningCommandName = "None";
    // This makes sure that the autonomous stops running when
    // teleop starts running. If you want the autonomous to
    // continue until interrupted by another command, remove
    // this line or comment it out.
    if (startPositionSelected != 0) {
      for (int i = 0; i < numberOfAutonomousCommands; i++) {

        if (autonomousCommand[1] != null) {
          autonomousCommand[i].cancel();
          autonomousCommandDone[i] = false;
        }

      }
    }
  }

  /**
   * This function is called periodically during operator control.
   */
  @Override
  public void teleopPeriodic() {

    Scheduler.getInstance().run();
    if (doTeleopPosition) {
      positionTargetFt = SmartDashboard.getNumber("Target Feet", 5);
      positionFPS = SmartDashboard.getNumber("Position FPS", 12);
      new RobotDriveToTarget(positionTargetFt, positionFPS, false, 8).start();
      doTeleopPosition = false;
    }

    if (doTeleopOrient) {
      // sensors.resetGyro();

      angleTarget = SmartDashboard.getNumber("Target Angle", 90);
      if (SmartDashboard.getBoolean("ReverseOrient", false))
        angleTarget = -angleTarget;
      orientRate = SmartDashboard.getNumber("Orient Rate", .25);
      new RobotOrient(angleTarget, orientRate, true, 30).start();
      doTeleopOrient = false;
    }

    /**
     * 
     * 
     * 
     */

    if (doFileTrajectory) {

      testTrajectoryName = AutoChoosers.testTrajectoryChooser.getSelected();
      testTrajectoryDirection = AutoChoosers.trajectoryDirectionChooser.getSelected();
      if (activeTrajName != testTrajectoryName) {
        activeTrajectory[0] = BuildTrajectory.buildLeftFileName(useUsb, testTrajectoryName);
        activeTrajectory[1] = BuildTrajectory.buildRightFileName(useUsb, testTrajectoryName);
        activeTrajName = testTrajectoryName;
        SmartDashboard.putBoolean("FileOK", buildOK);
        Robot.logName = activeTrajName;
      } else
        buildOK = true;
      if (!buildOK) {

        doFileTrajectory = false;
        DriverStation.reportError("Error reading file", true);
      }
      activeTrajName = testTrajectoryName;
      SmartDashboard.putBoolean("FileOK", buildOK);
      Robot.logName = activeTrajName;
    }

    if ((doFileTrajectory && buildOK)) {
      if (useGainPrefs)
        constantsFromPrefs();
      else
        activeTrajectoryGains = TrajDict.getTrajGains(activeTrajName);
      int trajectoryDirectionChooser = AutoChoosers.trajectoryDirectionChooser.getSelected();
      invertY = SmartDashboard.getBoolean("InvertY", false);
      useGainPrefs = SmartDashboard.getBoolean("UseGainPrefs", true);

      switch (trajectoryDirectionChooser) {

      case 1:// move forward into field
        new PathfinderTrajectory(faceField, invertY).start();
        break;
      case 2:// move reverse into field
        new PathfinderTrajectory(!faceField, invertY).start();
        break;
      case 3:// move reverse to wall
        new PathfinderReverseTrajectory(faceField, invertY).start();

        break;
      case 4:// move forward to wall
        new PathfinderReverseTrajectory(!faceField, invertY).start();
        break;

      }

      trajectoryRunning = true;
      doTeleopTrajectory = false;
      doFileTrajectory = false;
      doFileTrajectory = false;
    }
    doFileTrajectory = false;

    updateStatus();
  }

  /**
   * This function is called periodically during test mode.
   */
  @Override
  public void testPeriodic() {

  }

  public void updateStatus() {
    driveTrain.updateStatus();
    limelightCamera.updateStatus();
    visionData.updateStatus();
    SmartDashboard.putBoolean("BuildInProg", buildInProgress);
    SmartDashboard.putBoolean("BuildOK", buildOK);
    SmartDashboard.putNumber("SecondHatchIndex", secondHatchIndex);
    SmartDashboard.putNumber("NmrAutoCmds", numberOfAutonomousCommands);
    SmartDashboard.putBoolean("AC0Dn", autonomousCommandDone[0]);
    SmartDashboard.putString("Running Command", runningCommandName);
    SmartDashboard.putBoolean("PosnRng", isPositioning);
    SmartDashboard.putBoolean("TrajRng", trajectoryRunning);
    SmartDashboard.putBoolean("OrientRng", orientRunning);
    SmartDashboard.putNumber("TrajLen", activeTrajectory == null ? 0 : activeTrajectory[0].length());
    SmartDashboard.putString("FileChosen", chosenFileName);
    SmartDashboard.putString("FileInBuffer", bufferTrajName);
    SmartDashboard.putString("Active Trajectory", activeTrajName);

    SmartDashboard.putNumber("AG0", activeTrajectoryGains[0]);
    SmartDashboard.putNumber("AG1", activeTrajectoryGains[1]);
    SmartDashboard.putNumber("AG2", activeTrajectoryGains[2]);
    SmartDashboard.putNumber("AG3", activeTrajectoryGains[3]);

  }

  private void constantsFromPrefs() {
    activeTrajectoryGains[0] = Pref.getPref("PathKp");// prefs.getDouble("PathP", 0);
    activeTrajectoryGains[1] = Pref.getPref("PathKd");// prefs.getDouble("PathD", 0);
    activeTrajectoryGains[2] = Pref.getPref("PathKa");// prefs.getDouble("PathA", 0);
    activeTrajectoryGains[3] = Pref.getPref("PathKt");// prefs.getDouble("PathTurn", 0);
  }

  public void readTrajFiles() {

    // currentLoader = new LoadFiles();

    // if (!firstTime)
    // oldLoader = currentLoader;
    // else oldLoader.valid=false;
    // firstTime = true;
    Thread reader = new Thread(currentLoader);

    reader.setDaemon(true);
    if (!startSettingsDone)
      reader.start();
  }
}
