/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package tech.team1781.infiniteRecharge;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.Joystick;

/**
 * Add your docs here.
 */
public class Climber {
    CANSparkMax climber;
    WPI_TalonSRX deployMotor;
    Joystick controlStick;

    public Climber(Joystick _controlStick) {
        climber = new CANSparkMax(ConfigMap.CLIMB_MOTOR, MotorType.kBrushless);
        deployMotor = new WPI_TalonSRX(ConfigMap.HOOK_DEPLOY);
        controlStick = _controlStick;
    }

    public void update() {
        if (controlStick.getRawButton(ConfigMap.DEPLOY)) {
            deployHook();
        } else if (controlStick.getRawButton(ConfigMap.RETRACT)) {
            retractHook();
        } else {
            deployMotor.set(0);
        }
        climb();
    }

    public void deployHook() {
        deployMotor.set(1);
    }

    public void retractHook() {
        deployMotor.set(-1);
    }

    public void climb() {
        if (controlStick.getRawButton(ConfigMap.CLIMB)) {
            climber.set(-1);
        } else {
            climber.set(0);
        }
    }
}