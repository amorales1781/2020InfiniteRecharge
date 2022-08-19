/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package tech.team1781.infiniteRecharge;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;

import edu.wpi.first.wpilibj.AnalogInput;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.SpeedControllerGroup;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.controller.PIDController;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.shuffleboard.SimpleWidget;

public class Turret {
    ShuffleboardTab current = Shuffleboard.getTab("CurrentOutput");
    ShuffleboardTab shootingData = Shuffleboard.getTab("Extra Data For Shooter");
    SimpleWidget leftMotor, rightMotor, rotateMotor;
    SimpleWidget shootingRPM, isAtRPM, turretEncoderData;
    SimpleWidget leftLimitSwitchData, rightLimitSwitchData;
    Camera limeLight;
    Encoder shootingEncoder, turretEncoder;
    double requestRPM;
    double shootingVelocity;
    AnalogInput hoodPosition = new AnalogInput(0);

    double hoodTargetPosition = ConfigMap.GREEN_HOOD;
    boolean shootHighSpeed = true;

    private WPI_TalonSRX leftShooter, rightShooter, turretRotate, turretHood;
    DigitalInput leftLimitSwitch, rightLimitSwitch;

    SpeedControllerGroup shooting;

    Joystick joy;

    PIDController turretPID;

    double tkp = 0.00015;
    double tki = 0;
    double tkd = 0;

    PIDController shootingPID;

    double skp = 0.01;
    double ski = 0;
    double skd = 0;

    boolean isCalibrated = true;
    public Timer shootingTimer;
    float shootingSpeed = 0;
    double Kp = -0.09f;
    double min_command = 0.0001f;
    double rotateSpeed;

    boolean shooterAtSpeed;

    public Turret(Joystick _joy) {
        shootingTimer = new Timer();
        leftShooter = new WPI_TalonSRX(ConfigMap.TURRET_SHOOTING_LEFT);
        rightShooter = new WPI_TalonSRX(ConfigMap.TURRET_SHOOTING_RIGHT);

        turretHood = new WPI_TalonSRX(ConfigMap.TURRET_HOOD);

        rightShooter.setInverted(true);
        leftShooter.setInverted(false);

        turretRotate = new WPI_TalonSRX(ConfigMap.TURRET_ROTATION);

        joy = _joy;

        shooting = new SpeedControllerGroup(leftShooter, rightShooter);

        shootingEncoder = new Encoder(ConfigMap.SHOOTER_ENCODER_DIO1, ConfigMap.SHOOTER_ENCODER_DIO2);
        turretEncoder = new Encoder(ConfigMap.TURRET_ENCODER_DIO1, ConfigMap.TURRET_ENCODER_DIO2);

        leftLimitSwitch = new DigitalInput(ConfigMap.LEFT_SHOOT_LIMIT_SWITCH);
        rightLimitSwitch = new DigitalInput(ConfigMap.RIGHT_SHOOT_LIMIT_SWITCH);

        leftMotor = current.add("Left Shooting Motor Current", 0.0);
        rightMotor = current.add("Right Shooting Motor Current", 0.0);
        rotateMotor = current.add("Shooting Rotate Motor Current", 0.0);
        shootingRPM = shootingData.add("Speed of Motor", 0.0);
        turretEncoderData = shootingData.add("encoder count of turret", 0.0);
        leftLimitSwitchData = shootingData.add("left limit switch", false);
        rightLimitSwitchData = shootingData.add("right limit switch", false);
        isAtRPM = shootingData.add("Is at RPM", false);

        turretPID = new PIDController(tkp, tki, tkd);

        shootingPID = new PIDController(tkp, tki, tkd);

        hoodTargetPosition = ConfigMap.GREEN_HOOD;
    }

    public void shoot() {
        if (!shootHighSpeed)
            shooting.set(-0.6875);
        else
            shooting.set(-1.0f);
    }

    public void setShootHighSpeed(boolean isHighSpeed) {
        shootHighSpeed = isHighSpeed;
    }

    public void setHoodTarget(double targetValue) {
        hoodTargetPosition = targetValue;
    }

    public void update() {

        double hoodError = hoodPosition.getAverageVoltage() - hoodTargetPosition;

        // Acceptable threshold
        if (Math.abs(hoodError) < 0.005)
            hoodIdle();
        else if (hoodError > 0)
            hoodAdjust(false);
        else
            hoodAdjust(true);

    }

    public boolean atSpeed() {
        if (shootHighSpeed) {
            if (Math.abs(shootingEncoder.getRate() / 8192) > 29)
                return true;
        } else {
            if (Math.abs(shootingEncoder.getRate() / 8192) > 20)
                return true;
        }
        return false;
    }

    public double getSpeed() {
        return shootingEncoder.getRate() / 8192;
    }

    public void setShootingSpeed(double _rpm) {
        requestRPM = _rpm;
        // shooting.set(constrain(shootingPID.calculate(shootingEncoder.getRate(),
        // _rpm), -1.0, 1.0));
        if (_rpm - 25000 > shootingEncoder.getRate()) {
            shootingVelocity += .01;
        } else if (_rpm + 25000 < shootingEncoder.getRate()) {
            shootingVelocity -= .01;
        } else if (_rpm - 25000 < shootingEncoder.getRate() && _rpm + 25000 > shootingEncoder.getRate()) {
            shootingVelocity += 0;
        } else if (_rpm == 0) {
            shootingVelocity = 0;
        }
        shooting.set(shootingVelocity);
    }

    public boolean isAtSpeed(double _rpm) {
        if (shootingEncoder.getRate() < _rpm + 20000 && shootingEncoder.getRate() > _rpm - 20000) {
            return true;
        }
        return false;
    }

    public double voltsToRPM(double _volts) {
        double rpm = _volts * 250000;
        return rpm;
    }

    public void stopShooting() {
        shooting.set(0);
    }

    public void reverseShooting() {
        shooting.set(0.5);
    }

    public void autoAim(double portTarget) {
        double heading_error = -portTarget;

        if (portTarget > 1.0) {
            rotateSpeed = Kp * heading_error + min_command;
        } else if (portTarget < -1.0) {
            rotateSpeed = Kp * heading_error - min_command;
        }
        if (rotateSpeed > 0) {
            aimLeft(rotateSpeed);
        } else {
            aimRight(rotateSpeed);
        }

    }

    public void aimLeft(double speed) {
        if (leftLimitSwitch.get()) {
            if (isCalibrated && turretEncoder.get() < .9 * 11000) {
                turretRotate.set(speed);
            } else
                turretRotate.set(speed * 0.5);
        } else
            stopTurret();
    }

    public void aimRight(double speed) {
        if (!rightLimitSwitch.get()) {
            isCalibrated = true;
            turretEncoder.reset();
        }

        if (rightLimitSwitch.get()) {
            if (isCalibrated && turretEncoder.get() > .1 * 11000) {
                turretRotate.set(speed);
            } else
                turretRotate.set(speed * 0.5);
        } else
            stopTurret();
    }

    public boolean isCalibrated() {
        return isCalibrated;
    }

    public void stopTurret() {
        turretRotate.set(0);
    }

    public void sendData() {
        // System.out.println("Shooting RPM");
        // System.out.println(shootingEncoder.get());
        // System.out.println("request RPM" + requestRPM);
        // System.out.println("current RPM" + shootingEncoder.getRate());
        shootingRPM.getEntry().setDouble(shootingEncoder.getRate());
        turretEncoderData.getEntry().setDouble(turretEncoder.get());
        leftLimitSwitchData.getEntry().setBoolean(leftLimitSwitch.get());
        rightLimitSwitchData.getEntry().setBoolean(rightLimitSwitch.get());
        isAtRPM.getEntry().setBoolean(isAtSpeed(requestRPM));
    }

    public void resetTurretEncoder() {
        if (!isCalibrated && rightLimitSwitch.get()) {
            turretRotate.set(.5);
        } else if (!isCalibrated && !rightLimitSwitch.get()) {
            isCalibrated = true;
            turretEncoder.reset();
        }
    }

    public void resetData() {
        isCalibrated = false;
    }

    public double constrain(double value, double min, double max) {
        return (value > max) ? max : (value < min ? min : value);
    }

    public void rotateToAngle(double _angle) {
        if (isCalibrated && rightLimitSwitch.get() && leftLimitSwitch.get()) {
            double angle = (_angle / 310) * 11000;
            turretRotate.set(turretPID.calculate(turretEncoder.get(), angle) * -1);
        } else {
            turretRotate.set(0);
        }
    }

    public void hoodAdjust(boolean forward) {
        if (forward && hoodPosition.getAverageVoltage() < 4.5)
            turretHood.set(0.1f);
        else if (!forward && hoodPosition.getAverageVoltage() > .3)
            turretHood.set(-0.1f);
        else {
            hoodIdle();
            // System.out.println("Hood vals: " + hoodPosition.getAverageVoltage() + "," +
            // hoodTargetPosition);
        }
    }

    public void hoodIdle() {
        turretHood.set(0);
        //System.out.println("Hood Adjust value: " + hoodPosition.getAverageVoltage());
    }

}
