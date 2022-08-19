/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package tech.team1781.infiniteRecharge;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.SpeedControllerGroup;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.controller.PIDController;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.shuffleboard.SimpleWidget;

import com.revrobotics.CANEncoder;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.wpilibj.DoubleSolenoid;

import com.kauailabs.navx.frc.AHRS;

/**
 * Add your docs here.
 */
public class DriveSystem {
    CANEncoder front_right_Encoder, front_left_Encoder, back_right_Encoder, back_left_Encoder;
    CANSparkMax front_right, front_left, back_right, back_left;
    SpeedControllerGroup m_left, m_right;
    public DifferentialDrive robotDrive;
    DoubleSolenoid driveSolinoid;
    double rotateSpeed;

    double requestedRotation, requestedThrust, currentRotation, currentThrust;

    Joystick driveStick;

    ShuffleboardTab driveSystemType = Shuffleboard.getTab("Drive System Type");
    ShuffleboardTab encoderValues = Shuffleboard.getTab("Encoder Values");
    ShuffleboardTab current = Shuffleboard.getTab("CurrentOutput");
    ShuffleboardTab NavX = Shuffleboard.getTab("NavX");
    ShuffleboardTab PID = Shuffleboard.getTab("PID");
    SimpleWidget PIDValue, currentValue, encoderAverage, distanceAway;
    SimpleWidget spark1, spark2, spark3, spark4, total, left, right;
    SimpleWidget fle, ble, fre, bre;
    SimpleWidget navP, navY, navR;

    NetworkTableEntry driveType = driveSystemType.add("Curvature  |  Arcade     ", false).getEntry();
    // float totalAmps;
    Timer driveTimer;

    AHRS driveNavX;

    PIDController drivePID;
    double p = 0.01;
    double i;
    double d;
    double kp = 0.02;
    double ki = 0.01;
    double kd = 0;
    double currentSpeed = 0;
    double currentChange = 0.1f;
    double PIDRequestSpeed = 0;
    PIDController curveReductionPID, straightPID;
    double kpCurve = 0.02;
    double kiCurve = 0;
    double kdCurve = 0;
    double rotation;
    double initEncoderCount, changeOfEncoder, initTime, changeOfTime;

    public boolean atLocation;

    public DriveSystem(Joystick _driveStick, AHRS _navX) {
        driveStick = _driveStick;
        driveNavX = _navX;
        driveTimer = new Timer();

        front_right = new CANSparkMax(ConfigMap.DRIVE_FRONT_RIGHT, MotorType.kBrushless);
        back_right = new CANSparkMax(ConfigMap.DRIVE_BACK_RIGHT, MotorType.kBrushless);

        front_right.setOpenLoopRampRate(0.1);
        back_right.setOpenLoopRampRate(0.1);

        front_right.setInverted(false);
        back_right.setInverted(false);

        m_right = new SpeedControllerGroup(front_right, back_right);

        back_left = new CANSparkMax(ConfigMap.DRIVE_BACK_LEFT, MotorType.kBrushless);
        front_left = new CANSparkMax(ConfigMap.DRIVE_FRONT_LEFT, MotorType.kBrushless);

        front_left.setOpenLoopRampRate(0.1);
        back_left.setOpenLoopRampRate(0.1);

        front_left.setInverted(false);
        back_left.setInverted(false);

        m_left = new SpeedControllerGroup(front_left, back_left);

        robotDrive = new DifferentialDrive(m_left, m_right);

        driveSolinoid = new DoubleSolenoid(ConfigMap.PCM_CanID, ConfigMap.DriveSolenoidChannelForward,
                ConfigMap.DriveSolenoidChannelReverse);

        drivePID = new PIDController(kp, ki, kd);

        straightPID = new PIDController(p, i, d);

        curveReductionPID = new PIDController(kpCurve, kiCurve, kdCurve);

        front_left_Encoder = front_left.getEncoder();
        back_left_Encoder = back_left.getEncoder();
        front_right_Encoder = front_right.getEncoder();
        back_right_Encoder = back_right.getEncoder();

        fle = encoderValues.add("Front Left Encoders", 0.0);
        ble = encoderValues.add("Back Left Encoders", 0.0);
        fre = encoderValues.add("Front Right Encoders", 0.0);
        bre = encoderValues.add("Back Right Encoders", 0.0);

        navP = NavX.add("Pitch", driveNavX.getPitch());
        navY = NavX.add("Yaw", driveNavX.getYaw());
        navR = NavX.add("Roll", driveNavX.getRoll());

        spark1 = current.add("Spark 21 (FR)", 0.0);
        spark3 = current.add("Spark 23 (BR)", 0.0);
        left = current.add("Right Total Current", 0.0);

        spark2 = current.add("Spark 22 (FL)", 0.0);
        spark4 = current.add("Spark 24 (BL)", 0.0);
        right = current.add("Left Total Current", 0.0);

        total = current.add("Total Current", 0.0);

        PIDValue = PID.add("Request (PID) Speed", 0.0);
        currentValue = PID.add("Current Speed", 0.0);
        encoderAverage = PID.add("Average of all encoders", 0.0);
        distanceAway = PID.add("Distance Error", 0.0);

        driveTimer.start();

    }

    public void update() {

        if (driveStick.getRawButton(ConfigMap.LOW_GEAR_BUTTON)) {
            switchToLowGear();
        } else {
            switchToHighGear();
        }
        
        if(driveStick.getRawButton(ConfigMap.CIRCLE_LEFT_BUTTON))
        {
            robotDrive.arcadeDrive(0.75, -0.63);
        }else if (driveStick.getRawButton(ConfigMap.CIRCLE_RIGHT_BUTTON))
        {
            robotDrive.arcadeDrive(0.75, 0.625);
        }else 
            rampUpDriving();
  
        sendData();
    }

    public float constrain(float value, float min, float max) {

        return Math.min(Math.max(value, min), max);

    }

    public double getMotorSpeed() {
        return m_left.get();
    }

    public void setOpenLoopRampRate(double _rate) {
        front_right.setOpenLoopRampRate(_rate);
        back_right.setOpenLoopRampRate(_rate);
        front_left.setOpenLoopRampRate(_rate);
        back_left.setOpenLoopRampRate(_rate);
    }

    public void switchToLowGear() {
        driveSolinoid.set(DoubleSolenoid.Value.kForward);
    }

    public void switchToHighGear() {
        driveSolinoid.set(DoubleSolenoid.Value.kReverse);
    }

    public void targetLockOn(double targetLocation) {
        if (targetLocation == -1) {
            robotDrive.arcadeDrive(0.5, 0.0);
        } else {
            if (targetLocation > 350 / 2 - 30 && targetLocation < 350 / 2 + 30) {
                robotDrive.arcadeDrive(0.5, 0);
            } else if (targetLocation > 350 / 2 + 30) {
                robotDrive.arcadeDrive(0.5, calculateTurnSpeed(targetLocation) / 10 + 0.3);
            } else if (targetLocation < 350 / 2 - 30) {
                robotDrive.arcadeDrive(0.5, calculateTurnSpeed(targetLocation) / 10 - 0.3);
            }
        }
    }

    public double calculateTurnSpeed(double x) {
        if (x > 175) {
            rotateSpeed = (x - 200) / 21;
        } else if (x < 175) {
            rotateSpeed = (x - 150) / 21;
        }
        rotateSpeed = constrain(rotateSpeed, 0.3, 0.6);
        return rotateSpeed;
    }

    public void driveStraight(double _speed) {
        if (driveNavX.getYaw() < 2.5 && driveNavX.getYaw() > -2.5) {
            robotDrive.arcadeDrive(_speed, 0);
        } else if (driveNavX.getYaw() > 2.5) {
            robotDrive.arcadeDrive(_speed, -0.2);
        } else if (driveNavX.getYaw() < -2.5) {
            robotDrive.arcadeDrive(_speed, 0.2);
        }
        // rotation = straightPID.calculate(driveNavX.getYaw(), 0);
    }

    public double getEncoderPerSeconds() {

        changeOfEncoder = front_left_Encoder.getPosition() - initEncoderCount;
        changeOfTime = driveTimer.get() - initTime;
        initEncoderCount = front_left_Encoder.getPosition();
        initTime = driveTimer.get();

        return changeOfEncoder / changeOfTime;
    }

    public void getEncoderCount() {
        System.out.println("-------------------");
        System.out.println("FrontleftEncoder:");
        System.out.println(front_left_Encoder.getPosition());
        System.out.println("BackleftEncoder:");
        System.out.println(back_left_Encoder.getPosition());
        System.out.println("FrontRightEncoder:");
        System.out.println(front_right_Encoder.getPosition());
        System.out.println("BackRightEncoder:");
        System.out.println(back_right_Encoder.getPosition());
    }

    public void move(float distance) {
        // 1 meter = -35 rotations

        PIDRequestSpeed = drivePID.calculate((-35) * distance
                - calculateAverage(calculateAverage(front_left_Encoder.getPosition(), back_left_Encoder.getPosition()),
                        calculateAverage(front_left_Encoder.getPosition(), back_left_Encoder.getPosition())));

        setSpeed(PIDRequestSpeed);
        robotDrive.arcadeDrive(currentSpeed, 0);
        distanceAway.getEntry()
                .setDouble((-35) * distance - calculateAverage(
                        calculateAverage(front_left_Encoder.getPosition(), back_left_Encoder.getPosition()),
                        calculateAverage(front_left_Encoder.getPosition(), back_left_Encoder.getPosition())));

    }

    public void driveDistanceWithAdjustment(double adjustment)
    {
        
    }

    public void setSpeed(double requestSpeed) {
        // currentSpeed = front_left.get();
        if (currentSpeed < requestSpeed && currentSpeed < 1) {
            currentSpeed += currentChange;
        } else if (currentSpeed > requestSpeed && currentSpeed > -1) {
            currentSpeed -= currentChange;
        } else if (requestSpeed == 0) {
            currentSpeed = 0;
        }
    }

    public double calculateAverage(double num1, double num2) {
        double sum = num1 + num2;
        return sum / 2;
    }

    public void resetData() {
        front_left_Encoder.setPosition(0);
        back_left_Encoder.setPosition(0);
        front_right_Encoder.setPosition(0);
        back_right_Encoder.setPosition(0);
        zeroNavX();
        currentSpeed = 0;
    }

    public void zeroNavX() {
        driveNavX.zeroYaw();
    }

    public void sendData() {
        fle.getEntry().setDouble(front_left_Encoder.getPosition());
        ble.getEntry().setDouble(back_left_Encoder.getPosition());
        fre.getEntry().setDouble(front_right_Encoder.getPosition());
        bre.getEntry().setDouble(back_right_Encoder.getPosition());

        navP.getEntry().setDouble(driveNavX.getPitch());
        navY.getEntry().setDouble(driveNavX.getYaw());
        navR.getEntry().setDouble(driveNavX.getRoll());

        // total.getEntry().setDouble(totalAmps);

        PIDValue.getEntry().setDouble(PIDRequestSpeed);
        currentValue.getEntry().setDouble(currentSpeed);
        encoderAverage.getEntry()
                .setDouble(calculateAverage(
                        calculateAverage(front_left_Encoder.getPosition(), back_left_Encoder.getPosition()),
                        calculateAverage(front_left_Encoder.getPosition(), back_left_Encoder.getPosition())));
    }

    public double constrain(double value, double min, double max) {
        return (value > max) ? max : (value < min ? min : value);
    }

    public double getEncoderPosition() {
        return back_left_Encoder.getPosition();
    }

    public void rampUpDriving() {
        requestedThrust = -driveStick.getY();
        requestedRotation = driveStick.getZ();

        requestedRotation = requestedRotation * Math.abs(driveStick.getThrottle());

        double errThurst = currentThrust - requestedThrust;
        double errRotation = currentRotation - requestedRotation;

        if (Math.abs(errThurst) > .1) {
            currentThrust -= errThurst / 40;
        } else {
            currentThrust = requestedThrust;
        }

        if (Math.abs(errRotation) > 0.1) {
            currentRotation -= errRotation / 15;
        } else
            currentRotation = requestedRotation;
            
        robotDrive.arcadeDrive(currentThrust, currentRotation);

    }

    public void setCoast() {
        front_right.setIdleMode(IdleMode.kCoast);
        front_left.setIdleMode(IdleMode.kCoast);
        back_right.setIdleMode(IdleMode.kCoast);
        back_left.setIdleMode(IdleMode.kCoast);
    }

    public void setBrake() {
        front_right.setIdleMode(IdleMode.kBrake);
        front_left.setIdleMode(IdleMode.kBrake);
        back_right.setIdleMode(IdleMode.kBrake);
        back_left.setIdleMode(IdleMode.kBrake);
    }

}
