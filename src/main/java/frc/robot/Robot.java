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
import frc.robot.commands.Motion.RobotOrient;
import frc.robot.commands.Motion.RobotOrientToVision;
import frc.robot.commands.Motion.RobotDriveToTarget;

import frc.robot.commands.Auto.*;
import frc.robot.commands.Trajectories.PickAndRunTrajectory;
import frc.robot.BuildTrajectory;
import frc.robot.TrajDict;
import frc.robot.subsystems.DriveTrain;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.PowerPanel;
import frc.robot.subsystems.RobotRotate;
import frc.robot.subsystems.RotateToVision;

import frc.robot.subsystems.AirCompressor;
import frc.robot.subsystems.GamePieceHandler;
import jaci.pathfinder.Trajectory;
import frc.robot.AutoChoosers;
import frc.robot.VisionData;
import frc.robot.AutoCommands;
import frc.robot.LimeLight;
import frc.robot.LoadFiles;
import frc.robot.PathfinderNotifier;
import frc.robot.PathfinderReverseNotifier;
import frc.robot.LimelightControlMode.*;

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
  public static RotateToVision rotateToVision;
  public static Elevator elevator;
  public static GamePieceHandler gph;
  public static AirCompressor airCompressor;
  public static PowerPanel pdp;
  public static SimpleCSVLogger2194 simpleCSVLogger2194;
  public static LimeLight limelightCamera;
  public static VisionData visionData;
  public static AutoChoosers autoChoosers;
  public static String chosenFileName = "Dummy";
  public static OI m_oi;
  public static Preferences prefs;
  public static BuildTrajectory buildTrajectory;
  public static int maxCommands = 20;
  public static Command[] autonomousCommand;
  public static double[] commandTimes = new double[maxCommands];
  public static Command autoTimeDelayCommand;
  double autoTimeDelaySeconds;

  public static boolean autonomousCommandDone;
  public static String[] autonomousCommandName;
  public static int runningAutoCommand;;

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
  public static boolean doTeleopVisionOrient;
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

  public static Trajectory activeLeftTrajectory;
  public static Trajectory activeRightTrajectory;
  public static String activeTrajName = "Empty";
  public static Trajectory[] leftBufferTrajectory = new Trajectory[8];
  public static Trajectory[] rightBufferTrajectory = new Trajectory[8];
  public static String[] bufferTrajectoryName = { "0", "1", "2", "3", "4", "5", "7" };

  public static int secondHatchIndex;
  public static String bufferTrajName = "Empty";
  public static String testTrajectoryName;
  public static int testTrajectorySelection;
  public static int testTrajectoryDirection;

  public static double[] activeTrajectoryGains = { 0, 0, 0, 0 };
  public static double[][] bufferTrajectoryGains = new double[6][4];

  public static boolean revTraj;

  public static boolean doFileTrajectory;
  public static boolean trajectoryRunning;
  public static boolean buildOK;
  public static int currentTrajectorySegment;
  public static boolean trajectoryPulse;

  public static String names = "Step,LeftCmd,LeftFt,RightCmd,RightFt,AngleCmd,AngleAct, LeftSegVel,left,ActLeftVel,RightSegVel,right,ActRightVel,turn\n";
  public static String units = "NumberFT,FT,FT,FT,FT,Deg,Deg,pct,pct,pct,pct,pct,pct,pct\n";
  public static boolean createTrajectoryRunFile = true;
  public static boolean createMultipleTrajectoryRunFile;
  public static double trajectoryX;
  public static double trajectoryY;
  public static double trajectoryAngle;
  public static String trajFileName;
  public static String logName;
  public static boolean robotMoveReverse;
  int test99;
  public static boolean buildInProgress;
  public static int startPositionSelected = 0;
  public static int secondHatchSelected = 0;
  public static boolean useUsb = false;
  public static boolean faceField;
  public static boolean invertY;
  public static boolean towardsFieldTrajectory;
  public static boolean trajectoriesLoaded;

  public static int numberOfAutonomousCommands;
  public static boolean startSettingPB = false;
  public static boolean startSettingsDone = false;
  public static boolean setAutoStartPB = false;
  public static boolean autoSetupRunning = false;
  public static boolean setAutoStartDone = false;
  public static boolean readingRunning = false;

  // LoadAllFiles currentLoader = new LoadAllFiles();
  LoadFiles currentLoader = new LoadFiles();
  boolean wasRunning = false;
  public static String runningCommandName = "None";
  public static double readThreadStartTime = 0.;
  public static boolean fileError;
  private int scanCounter;
  public static double autoStartTime;
  public static boolean cycleHold;
  public static boolean autoAbort;
  public static double sideAngle;
  public static boolean createDriveRunFile = true;
  public static boolean createElevatorRunFile = true;
  public static boolean createVisionRunFile = true;
  public static String driveLogName = "/U" + "/data_capturesDS19/Drive/Drive";
  public static String driveUniqueLogName;
  public static String trajectoryLogName = "/U" + "/data_capturesDS19/Trajectory/T";
  public static String trajectoryUniqueLogName;
  public static boolean useVisionComp;
  public static double activeMotionComp;
  public static boolean autoRunning;
  private double commandStartTime;
  public static boolean limelightOnEnd = true;

  /**
   * This function is run when the robot is first started up and should be used
   * for any initialization code.
   */
  @Override
  public void robotInit() {
    driveTrain = new DriveTrain();
    robotRotate = new RobotRotate();
    rotateToVision = new RotateToVision();
    elevator = new Elevator();

    airCompressor = new AirCompressor();
    pdp = new PowerPanel();
    gph = new GamePieceHandler();
    m_oi = new OI();
    simpleCSVLogger2194 = new SimpleCSVLogger2194();
    limelightCamera = new LimeLight();
    visionData = new VisionData();
    buildTrajectory = new BuildTrajectory();

    autoChoosers = new AutoChoosers();
    autonomousCommand = new Command[maxCommands];
    autonomousCommandName = new String[maxCommands];

    prefs = Preferences.getInstance();
    // Pref.deleteAllPrefs();
    // Pref.deleteUnused();
    // Pref.addMissing();
    SmartDashboard.putData(driveTrain);

    Timer.delay(.02);
    SmartDashboard.putNumber("Target Feet", 5);
    SmartDashboard.putNumber("Position FPS", 5);
    SmartDashboard.putNumber("Target Angle", angleTarget);
    Timer.delay(.02);
    SmartDashboard.putNumber("Orient Rate", orientRate);
    Timer.delay(.02);
    SmartDashboard.putBoolean("ReverseOrient", false);
    Timer.delay(.02);
    SmartDashboard.putBoolean("ReverseTrajectory", false);
    Timer.delay(.02);
    SmartDashboard.putBoolean("UseGainPrefs", true);
    Timer.delay(.02);
    SmartDashboard.putBoolean("UseUSBTraj", false);
    Timer.delay(.02);
    SmartDashboard.putBoolean("SetAuto", false);
    Timer.delay(.02);
    SmartDashboard.putBoolean("StartSet", false);
    Timer.delay(.02);
    SmartDashboard.putBoolean("CreateTrajFile", false);
    SmartDashboard.putData(driveTrain);
    SmartDashboard.putData(elevator);
    SmartDashboard.putData(robotRotate);
    SmartDashboard.putData(gph);
    SmartDashboard.putData(airCompressor);
    SmartDashboard.putData(pdp);
    SmartDashboard.putData(Scheduler.getInstance());

    bufferTrajectoryGains[0] = activeTrajectoryGains;
    startPositionSelected = 0;
    elevator.holdPositionInches = 0;

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
    Scheduler.getInstance().run();
    cancelAllAuto();

  }

  @Override
  public void disabledPeriodic() {

    readTrajFiles();

    // if (m_oi.gamepad.getButtonStateA()) {
    setUpAutoStart();
    Robot.gph.retractPusher();
    Robot.gph.gripHatchPanel();
    Robot.gph.retractHatchPanel();
    Robot.gph.secondRetractHatchPanel();

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
    autoStartTime = Timer.getFPGATimestamp();
    commandStartTime = autoStartTime;
    // setUpAutoStart();

    //
    if (autonomousCommand[0] != null && AutoChoosers.startPositionChooser.getSelected() != 0) {
      autonomousCommand[0].start();
      runningAutoCommand = 0;
      autoRunning = true;
    }
  }

  /**
   * This function is called periodically during autonomous.
   */
  @Override
  public void autonomousPeriodic() {
    Scheduler.getInstance().run();

    if (m_oi.abortAuto.get()) {
      cancelAllAuto();
    }
    if (m_oi.gamepad.getButtonStateA())
      cycleHold = true;
    if (m_oi.gamepad.getButtonStateB())
      cycleHold = false;

    if (autonomousCommandDone && numberOfAutonomousCommands > runningAutoCommand) {

      commandTimes[runningAutoCommand] = Timer.getFPGATimestamp() - commandStartTime;
      SmartDashboard.putNumber("CMDTime" + String.valueOf(runningAutoCommand), commandTimes[runningAutoCommand]);
      if (!cycleHold) {
        autonomousCommandDone = false;
        commandStartTime = Timer.getFPGATimestamp();
        runningAutoCommand++;
        if (autonomousCommand[runningAutoCommand] != null)
          autonomousCommand[runningAutoCommand].start();
        runningCommandName = autonomousCommandName[runningAutoCommand];
      }
    }
    if ((startPositionSelected == 1 || startPositionSelected == 4) && runningAutoCommand == 2 && trajectoryPulse) {
      limelightCamera.setLEDMode(LedMode.kforceOn);

    }
    if ((startPositionSelected == 1 || startPositionSelected == 4) && secondHatchSelected == 3
        && runningAutoCommand == 12 && trajectoryPulse) {
      limelightCamera.setLEDMode(LedMode.kforceOn);
    }

    if ((startPositionSelected == 2 || startPositionSelected == 3) && secondHatchSelected == 3
        && runningAutoCommand == 9 && trajectoryPulse) {
      limelightCamera.setLEDMode(LedMode.kforceOn);
      SmartDashboard.putNumber("SPPS1", startPositionSelected);
    }

    if (runningAutoCommand > numberOfAutonomousCommands) {
      numberOfAutonomousCommands = 0;
      runningAutoCommand = 0;
      autonomousCommandDone = false;
      autoRunning = false;
    }
  }

  @Override
  public void teleopInit() {
    Robot.runningCommandName = "None";
    // This makes sure that the autonomous stops running when
    // teleop starts running. If you want the autonomous to
    // continue until interrupted by another command, remove
    // this line or comment it out.
    DriveTrain.gyroOffset = 0;
    PathfinderReverseNotifier.stopNotfier();
    PathfinderNotifier.stopNotfier();
    cancelAllAuto();

  }

  /**
   * This function is called periodically during operator control.
   */
  @Override
  public void teleopPeriodic() {

    Scheduler.getInstance().run();

    if (!DriverStation.getInstance().isFMSAttached()) {
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
        new RobotOrient(angleTarget, orientRate, true, 5).start();
        doTeleopOrient = false;
      }
      if (doTeleopVisionOrient) {

        orientRate = SmartDashboard.getNumber("Orient Rate", .25);
        new RobotOrientToVision(orientRate, 30).start();
        doTeleopVisionOrient = false;
      }

      if (doFileTrajectory) {
        createTrajectoryRunFile = SmartDashboard.getBoolean("CreateTrajFile", true);
        useUsb = SmartDashboard.getBoolean("UseUSBTraj", false);
        testTrajectorySelection = AutoChoosers.testTrajectoryChooser.getSelected();

        switch (testTrajectorySelection) {
        case 0:
          testTrajectoryName = TrajDict.outsideStartNames[0];
          towardsFieldTrajectory = true;
          faceField = false;
          invertY = false;
          useGainPrefs = true;
          break;
        case 1:
          testTrajectoryName = TrajDict.outsideStartNames[0];
          towardsFieldTrajectory = true;
          faceField = false;
          invertY = true;
          useGainPrefs = true;
          break;
        case 2:
          testTrajectoryName = TrajDict.secondHatchPickupNames[0];
          towardsFieldTrajectory = false;
          faceField = true;
          invertY = false;
          useGainPrefs = true;
          break;
        case 3:
          testTrajectoryName = TrajDict.secondHatchPickupNames[0];
          towardsFieldTrajectory = false;
          faceField = true;
          invertY = true;
          useGainPrefs = true;
          break;
        case 4:
          testTrajectoryName = TrajDict.secondHatchPickupNames[1];
          towardsFieldTrajectory = false;
          faceField = true;
          invertY = false;
          useGainPrefs = true;
          break;
        case 5:
          testTrajectoryName = TrajDict.secondHatchPickupNames[1];
          towardsFieldTrajectory = false;
          faceField = true;
          invertY = true;
          useGainPrefs = true;
          break;

        case 6:
          testTrajectoryName = TrajDict.secondHatchDeliveryNames[0];
          towardsFieldTrajectory = true;
          faceField = false;
          invertY = false;
          useGainPrefs = true;
          break;
        case 7:
          testTrajectoryName = TrajDict.secondHatchDeliveryNames[0];
          towardsFieldTrajectory = true;
          faceField = false;
          invertY = true;
          useGainPrefs = true;
          break;
        case 8:
          testTrajectoryName = TrajDict.secondHatchDeliveryNames[1];
          towardsFieldTrajectory = true;
          faceField = false;
          invertY = false;
          useGainPrefs = true;
          break;
        case 9:
          testTrajectoryName = TrajDict.secondHatchDeliveryNames[1];
          towardsFieldTrajectory = true;
          faceField = false;
          invertY = true;
          useGainPrefs = true;
          break;
        case 10:
          testTrajectoryName = TrajDict.secondHatchDeliveryNames[2];
          towardsFieldTrajectory = true;
          faceField = false;
          invertY = false;
          useGainPrefs = true;
          break;
        case 11:
          testTrajectoryName = TrajDict.secondHatchDeliveryNames[2];
          towardsFieldTrajectory = true;
          faceField = false;
          invertY = true;
          useGainPrefs = true;
          break;
        case 12:
          testTrajectoryName = TrajDict.secondHatchDeliveryNames[3];
          towardsFieldTrajectory = true;
          faceField = false;
          invertY = false;
          useGainPrefs = true;
          break;
        case 13:
          testTrajectoryName = TrajDict.secondHatchDeliveryNames[3];
          towardsFieldTrajectory = true;
          faceField = false;
          invertY = true;
          useGainPrefs = true;
          break;
        case 14:
          testTrajectoryName = "StraightOne";
          towardsFieldTrajectory = true;
          faceField = true;
          invertY = false;
          useGainPrefs = true;
          break;
        case 15:
          testTrajectoryName = "StraightOne";
          towardsFieldTrajectory = false;
          faceField = true;
          invertY = false;
          useGainPrefs = true;
          break;
        case 16:
          testTrajectoryName = "CurveOne";
          towardsFieldTrajectory = true;
          faceField = true;
          invertY = false;
          useGainPrefs = true;
          break;
        case 17:
          testTrajectoryName = "CurveOne";
          towardsFieldTrajectory = false;
          faceField = true;
          invertY = false;
          useGainPrefs = true;
          break;

        default:
          break;
        }
        if (activeTrajName != testTrajectoryName) {
          double startFiletime = Timer.getFPGATimestamp();
          activeLeftTrajectory = BuildTrajectory.buildLeftFileName(useUsb, testTrajectoryName);
          activeRightTrajectory = BuildTrajectory.buildRightFileName(useUsb, testTrajectoryName);
          SmartDashboard.putNumber("USBTime ", Timer.getFPGATimestamp() - startFiletime);
          activeTrajName = testTrajectoryName;
          SmartDashboard.putBoolean("FileOK", buildOK);
          logName = activeTrajName;
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

        useGainPrefs = SmartDashboard.getBoolean("UseGainPrefs", true);

        if (!SmartDashboard.getBoolean("ReverseTrajectory", false))
          new PickAndRunTrajectory(towardsFieldTrajectory, faceField, invertY).start();
        else
          new PickAndRunTrajectory(!towardsFieldTrajectory, faceField, invertY).start();

        trajectoryRunning = true;
        doFileTrajectory = false;
      }
      doFileTrajectory = false;

    }
  }

  /**
   * This function is called periodically during test mode.
   */
  @Override
  public void testPeriodic() {

  }

  public void updateStatus() {
    scanCounter++;

    switch (scanCounter) {

    case 1:
    robotUpdateStatus();
     break;

    case 2:
      limelightCamera.updateStatus();
      break;

    case 3:
      visionData.updateStatus();
      break;

    case 4:
      SmartDashboard.putNumber("SCCTR", scanCounter);
      driveTrain.updateStatus();
      break;

    case 5:
      elevator.updateStatus();
      break;

    case 6:
      gph.updateStatus();
      break;

      case 7:
      airCompressor.updateStatus();
      break;
      case 8:
      robotRotate.updateStatus();
      break;
      case 9:
      rotateToVision.updateStatus();
      break;
      case 10:
      visionData.updateStatus();
      break;

    default:
      if (scanCounter > 15)
        scanCounter = 0;
      break;
    }
  }

  private void robotUpdateStatus() {
    SmartDashboard.putBoolean("AutoHold", cycleHold);
    SmartDashboard.putBoolean("LAutoRng", autoRunning);
    SmartDashboard.putString("Running Cmd Name", runningCommandName);
    SmartDashboard.putBoolean("PosnRng", isPositioning);

    createUniqueLogName();
SmartDashboard.putBoolean("BB X", m_oi.buttonBox.getDPadLeft());
SmartDashboard.putNumber("BB 123L", m_oi.buttonBox.getLT());
SmartDashboard.putNumber("BB 123R", m_oi.buttonBox.getRT());
    if (AutoChoosers.debugChooser.getSelected() == 1) {
      SmartDashboard.putNumber("TrajStep", currentTrajectorySegment);
      SmartDashboard.putBoolean("TrajPLS", trajectoryPulse);
      SmartDashboard.putBoolean("AutoStepDone", autonomousCommandDone);
      SmartDashboard.putBoolean("BuildInProg", buildInProgress);
      SmartDashboard.putBoolean("BuildOK", buildOK);
      SmartDashboard.putNumber("SecondHatchIndex", secondHatchIndex);
      SmartDashboard.putNumber("Running Cmd Nmbr", runningAutoCommand);
      SmartDashboard.putBoolean("TrajRng", trajectoryRunning);
      SmartDashboard.putBoolean("OrientRng", orientRunning);
      SmartDashboard.putString("FileChosen", chosenFileName);
      SmartDashboard.putString("FileInBuffer", bufferTrajName);
      SmartDashboard.putString("Active Trajectory", activeTrajName);
      SmartDashboard.putBoolean("LogOpen", simpleCSVLogger2194.log_open);
      SmartDashboard.putBoolean("Invert Y", invertY);
      SmartDashboard.putBoolean("Face Field", faceField);
      SmartDashboard.putBoolean("TrajFileDo", doFileTrajectory);
      SmartDashboard.putNumber("AGKp", activeTrajectoryGains[0]);
      SmartDashboard.putNumber("AGKd", activeTrajectoryGains[1]);
      SmartDashboard.putNumber("AGKa", activeTrajectoryGains[2]);
      SmartDashboard.putNumber("AGKt", activeTrajectoryGains[3]);
    }
  }

  public static void cancelAllAuto() {
    autoRunning = false;
    autonomousCommandDone = false;
    cycleHold = false;
    Scheduler.getInstance().run();
    Scheduler.getInstance().removeAll();
    driveTrain.leftDriveOut(0);
    driveTrain.rightDriveOut(0);
    for (int i = 0; i < maxCommands; i++) {

      if (autonomousCommand[i] != null) {
        autonomousCommand[i].cancel();

      }

    }
  }

  private void constantsFromPrefs() {
    activeTrajectoryGains[0] = Pref.getPref("PathKp");
    activeTrajectoryGains[1] = Pref.getPref("PathKd");
    activeTrajectoryGains[2] = Pref.getPref("PathKa");
    activeTrajectoryGains[3] = Pref.getPref("PathKt");
  }

  private void resetBufferNames() {
    for (int i = 0; i < bufferTrajectoryName.length; i++) {
      bufferTrajectoryName[i] = "Not Used";

      SmartDashboard.putString("Buffer " + String.valueOf(i), Robot.bufferTrajectoryName[i]);
    }
  }

  private void resetCommandNames() {

    for (int i = 0; i < maxCommands; i++) {
      autonomousCommandName[i] = "Not Used";
      SmartDashboard.putString("AutoCommand " + String.valueOf(i), Robot.autonomousCommandName[i]);
    }
  }

  public void readTrajFiles() {
    startSettingPB = SmartDashboard.getBoolean("StartSet", false);
    useUsb = SmartDashboard.getBoolean("UseUSBTraj", false);
    SmartDashboard.putBoolean("StartSettingsDone", startSettingsDone);
    SmartDashboard.putBoolean("File Error", fileError);
    SmartDashboard.putBoolean("File Running", readingRunning);
    readingRunning = currentLoader.running;
    readThreadStartTime = Timer.getFPGATimestamp();
    if (startSettingPB && !readingRunning) {

      startSettingsDone = false;
      currentLoader = new LoadFiles();
      Thread reader = new Thread(currentLoader);
      reader.setDaemon(true);
      reader.start();
      wasRunning = true;
    }
    fileError = currentLoader.error && currentLoader.running || fileError & !startSettingPB;

    if (wasRunning && !readingRunning) {
      startSettingPB = false;
      SmartDashboard.putBoolean("StartSet", false);
      SD.putN4("ThreadTime", Timer.getFPGATimestamp() - readThreadStartTime);
      startSettingsDone = true;
      wasRunning = false;
      // setUpAutoStart();
    }
  }

  void setUpAutoStart() {
    if (startSettingsDone)
      setAutoStartPB = SmartDashboard.getBoolean("SetAuto", false);
    else
      SmartDashboard.putBoolean("SetAuto", false);
    SmartDashboard.putBoolean("AutoStartSetupDone", setAutoStartDone);
    SmartDashboard.putBoolean("AutoSetupRunning", autoSetupRunning);

    if (setAutoStartPB & startSettingsDone && !autoSetupRunning) {
      resetCommandNames();
      autoSetupRunning = true;
      autoTimeDelaySeconds = AutoChoosers.timeDelayChooser.getSelected();

      startPositionSelected = AutoChoosers.startPositionChooser.getSelected();
      if (startPositionSelected == 0)
        autoTimeDelaySeconds = 0;
      secondHatchSelected = AutoChoosers.secondHatchChooser.getSelected();

      autonomousCommand[0] = new AutoWait(autoTimeDelaySeconds);

      autonomousCommandName[0] = "0- Time Delay";
      if (!fileError && startPositionSelected != 0) {

        switch (startPositionSelected) {
        case 1:
          invertY = false;
          sideAngle = 0;
          numberOfAutonomousCommands = AutoCommands.setOutsideStart();
          break;
        case 2:
          invertY = false;
          sideAngle = 0;
          numberOfAutonomousCommands = AutoCommands.setMiddleStart();
          limelightCamera.setLEDMode(LedMode.kforceOn);
          break;
        case 3:
          invertY = true;
          sideAngle = 180;
          numberOfAutonomousCommands = AutoCommands.setMiddleStart();
          limelightCamera.setLEDMode(LedMode.kforceOn);
          break;
        case 4:
          invertY = true;
          sideAngle = 180;
          numberOfAutonomousCommands = AutoCommands.setOutsideStart();
          break;
        }
        if (secondHatchSelected != 0) {
          int numberOfPickUpSecondHatchCommands = AutoCommands.pickUpSecondHatch(startPositionSelected,
              numberOfAutonomousCommands);
          SmartDashboard.putNumber("NPU", numberOfPickUpSecondHatchCommands);
          numberOfAutonomousCommands = numberOfPickUpSecondHatchCommands;

          int numberOfDeliverSecondHatchCommands = AutoCommands.deliverSecondHatch(secondHatchSelected,
              numberOfAutonomousCommands);

          numberOfAutonomousCommands = numberOfDeliverSecondHatchCommands;
        }
        SmartDashboard.putNumber("NDU", numberOfAutonomousCommands);

        AutoCommands.updateStatus(numberOfAutonomousCommands);

        activeLeftTrajectory = leftBufferTrajectory[0];
        activeRightTrajectory = rightBufferTrajectory[0];
        activeTrajName = bufferTrajectoryName[0];

        activeTrajectoryGains = bufferTrajectoryGains[0];
        SmartDashboard.putNumber("NmrAutoCmds", numberOfAutonomousCommands);

        autoSetupRunning = false;
        setAutoStartDone = true;
        setAutoStartPB = false;
        SmartDashboard.putBoolean("SetAuto", false);
      }
    }
  }

  public static void createUniqueLogName() {
    double temp = (int) Timer.getFPGATimestamp();
    driveUniqueLogName = driveLogName + String.valueOf(temp) + ".csv";
    trajectoryUniqueLogName = trajectoryLogName + String.valueOf(temp);
  }
}
