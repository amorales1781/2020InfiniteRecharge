/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.Compressor;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.RobotDrive;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.shuffleboard.SimpleWidget;
import edu.wpi.first.wpilibj.SerialPort;
import tech.team1781.infiniteRecharge.DriveSystem;
import tech.team1781.infiniteRecharge.Shooter;
import tech.team1781.infiniteRecharge.Camera;
import tech.team1781.infiniteRecharge.Climber;
import edu.wpi.cscore.UsbCamera;
import edu.wpi.cscore.MjpegServer;
import edu.wpi.cscore.VideoMode;
import edu.wpi.first.cameraserver.*;

import tech.team1781.infiniteRecharge.ControlPanel;
import tech.team1781.infiniteRecharge.Conveyor;

import javax.lang.model.util.ElementScanner6;

import com.kauailabs.navx.frc.AHRS;

/**
 * The VM is configured to automatically run this class, and to call the
 * functions corresponding to each mode, as described in the IterativeRobot
 * documentation. If you change the name of this class or the package after
 * creating this project, you must also update the build.gradle file in the
 * project.
 */
public class Robot extends TimedRobot {
  Joystick pilot, coPilot;
  DriveSystem drive;
  Compressor compress;
  Conveyor conveyor;
  Climber climb;
  ControlPanel controlPanel;
  UsbCamera camera;
  MjpegServer cameraServer;
  Camera limeLight;
  Shooter shoot;
  AHRS navX;

  Timer autoTimer, teleTimer;

  ShuffleboardTab autonomous = Shuffleboard.getTab("Auto");
  NetworkTableInstance netInstance;
  NetworkTable table, limeTable;
  NetworkTableEntry xLoc, limeX, limeY, limeTargetArea, camMode, cellsCollected, turnPiOff, heartBeat; // location of
                                                                                                       // powercell
  double PowercellXLoc;
  SimpleWidget heartBeatCounter;
  NetworkTableEntry autoChoice1 = autonomous.add("Trench  |  Middle     ", false).getEntry();
  // NetworkTableEntry autoChoice2 = autonomous.add("False | True ",
  // false).getEntry();
  // NetworkTableEntry autoChoice3 = autonomous.add("False | True ",
  // false).getEntry();
  boolean auto1, auto2, auto3;

  int state = 0;

  String gameData;

  @Override
  public void robotInit() {
    limeLight = new Camera();
    navX = new AHRS(SerialPort.Port.kMXP);

    autoTimer = new Timer();

    pilot = new Joystick(0);
    coPilot = new Joystick(1);

    drive = new DriveSystem(pilot, navX);
    climb = new Climber(coPilot);
    // controlPanel = new ControlPanel(coPilot);
    shoot = new Shooter(pilot, coPilot, limeLight);

    compress = new Compressor(50);
    compress.start();
    //camera = CameraServer.getInstance().startAutomaticCapture();
    //camera.setVideoMode(VideoMode.PixelFormat.kMJPEG, 160, 120, 20);
    //camera.setExposureManual(10);
    netInstance = NetworkTableInstance.getDefault();
    table = netInstance.getTable("SmartDashboard");
    turnPiOff = table.getEntry("piPower");
    xLoc = table.getEntry("x");
    heartBeatCounter = autonomous.add("Raspi Vision Counter", 0.0);

    autoTimer.start();

    drive.resetData();
    drive.switchToLowGear();
    drive.setBrake();
    limeLight.turnLightsOn();
  }

  @Override
  public void robotPeriodic() {
    turnPiOff = table.getEntry("piPower");
    heartBeatCounter.getEntry().setDouble(heartBeat.getDouble(0));
  }

  @Override
  public void disabledInit() {
    super.disabledInit();
    heartBeat = table.getEntry("heartBeat");
    heartBeatCounter.getEntry().setDouble(heartBeat.getDouble(0));
    drive.setCoast();
  }

  @Override
  public void autonomousInit() {
    limeLight.setCameraMode(0);
    autoTimer.reset();
    autoTimer.start();
    shoot.resetTimer();
    shoot.startTimer();
    shoot.tiltCollectorForward();
    drive.switchToLowGear();
    drive.setOpenLoopRampRate(1.5);
    auto1 = autoChoice1.getBoolean(false);
    auto2 = autoChoice1.getBoolean(false);
    auto3 = autoChoice1.getBoolean(false);
    auto1=false;
    drive.zeroNavX();
    drive.resetData();
  }

  /**
   * This function is called periodically during autonomous.
   */
  @Override
  public void autonomousPeriodic() {
    // if (auto1) {
    //   centerAuto();
    // } else {
    //   alliedTrenchAuto();
    // }
    //alliedTrenchAuto();
    timedAuto();
    drive.sendData();
  }

  /**
   * This function is called periodically during operator control.
   */
  @Override
  public void teleopInit() {
    super.teleopInit();
    teleTimer = new Timer();
    teleTimer.start();
    turnPiOff.forceSetDouble(1.0);
    drive.switchToLowGear();
    drive.setOpenLoopRampRate(0.1);
    limeLight.setCameraMode(1);
  }

  @Override
  public void teleopPeriodic() {
    
    drive.update();
    drive.getEncoderPerSeconds();
    shoot.update();
    if (pilot.getRawButton(11)) {
      compress.clearAllPCMStickyFaults();
    }

    // stop compresssor for climbing to reserve amperage for climbing
    if (teleTimer.get() > 120.0) {
      //compress.stop();
      climb.update();
    }
  }

  /**
   * This function is called periodically during test mode.
   */
  @Override
  public void testPeriodic() {
    drive.switchToLowGear();
    double temp = xLoc.getDouble(-1);
    System.out.println( "Powercell:" + temp);

    if ((temp < (320/2) - 20) && temp > 0)
      drive.robotDrive.arcadeDrive(0, -0.3);
    else if ((temp > (320/2) + 20))
      drive.robotDrive.arcadeDrive(0, 0.3);
    else if (temp == -1){
      System.out.println("NADA");
      drive.robotDrive.arcadeDrive(0, 0);
    }
    else
      drive.robotDrive.arcadeDrive(.2, 0);


    // // drive.switchToLowGear();
    // xLoc = table.getEntry("x");
    // // cellsCollected = table.getEntry("cells");
    // // System.out.println(limeLight.getPortX());
    // shoot.lockOnPort(limeLight.getPortX());
    // // System.out.println("===========" + xLoc.getDouble(-1));
    // // System.out.println("-------" + cellsCollected.getDouble(0));
    // // drive.targetLockOn(xLoc.getDouble(-1));
  }

  public void alliedTrenchAuto() {
    switch (state) {
      case -1:
      shoot.calibrate();
      if(shoot.isCalibrated())
      {
        state = 0;
      }
        break;
      case 0: //locate port and start reving the shooter
        shoot.lockOnPort(limeLight.getPortX());
        shoot.revShooterToSpeed();
        if(shoot.lockedOn)
        {
          state = 1;
          shoot.resetTimer();
          shoot.startTimer();
        }
        break;
      case 1: //shoot the powercells
        drive.robotDrive.arcadeDrive(0, 0);
        shoot.revShooterToSpeed();
        shoot.shootPowercells();
        if (shoot.empty) {
          state = 2;
          shoot.resetTimer();
        }
        break;
      case 2: //drive forward to the 3rd powercell
        xLoc = table.getEntry("x");
        shoot.collect();
        shoot.stopUpperConvey();
        if(drive.getEncoderPosition() < 135){
          drive.driveStraight(0.4);
          if(drive.getEncoderPosition() > 130)
          {
            drive.driveStraight(0.0);
          }
        }
        if(drive.getMotorSpeed() < 0.25 && drive.getEncoderPosition() > 130){
          drive.robotDrive.arcadeDrive(0, 0);
          state = 3;
          shoot.dontCollect();
        }
        break;
      case 3: //drive back to init line 
        if(drive.getEncoderPosition() > 10){
          drive.driveStraight(-0.7);
          shoot.revShooterToSpeed();
        }else {
          drive.robotDrive.arcadeDrive(0, 0);
          state = 4;
          shoot.startTimer();
        }
        break;
      case 4: //lock onto the port
        shoot.lockOnPort(limeLight.getPortX());
        shoot.revShooterToSpeed();
        if(shoot.lockedOn)
        {
          state = 5;
        }
        break;
      case 5: //SHOOT EM!!!
        drive.robotDrive.arcadeDrive(0, 0);
        shoot.revShooterToSpeed();
        shoot.shootPowercells();
        break;
    }
  }

  public void centerAuto() {
    switch (state) {
      case 0:
        drive.robotDrive.arcadeDrive(0, 0);
        shoot.shootPowercells();
        if (shoot.empty) {
          state = 1;
        }
        break;
      case 1:
        shoot.dontCollect();
        shoot.stopUpperConvey();
        if (autoTimer.get() < 8) {
          drive.robotDrive.arcadeDrive(0.5, 0);
        } else
          drive.robotDrive.arcadeDrive(0, 0);
        break;
    }
  }

  public void oppositeTrench() {
    switch (state) {
      case 0: // Shoot all powercells
        shoot.shootPowercells();
        if (shoot.empty) {
          state = 1;
        }
        break;
      case 1: // Drive off autoLine
        drive.targetLockOn(xLoc.getDouble(-1));
        if (shoot.collectedAll) {
          state = 2;
          shoot.dontCollect();
        }
        break;
      case 2:
        drive.move(-2); // move out of enemy trench
        break;
    }
  }

  void timedAuto()
  {
    if(autoTimer.get() < 1.0)
    {
       shoot.preAutoShoot();
    }else if (autoTimer.get()< 2){
      shoot.autoShoot();
    }else if (autoTimer.get()< 4)
    {
      shoot.autoShoot();
      drive.driveStraight(.75);
     }
    else if (autoTimer.get()< 10)
    {
      shoot.autoCollect();
      drive.driveStraight(.5);
    }else if (autoTimer.get()< 12)
    {
      shoot.autoCollect();
      drive.driveStraight(-.75);
    }else if (autoTimer.get()< 15)
    {
      shoot.autoShoot();
    }else 
    {
      drive.driveStraight(0);
      shoot.idle();
    }
  }

}
