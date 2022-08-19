/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package tech.team1781.infiniteRecharge;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;

import edu.wpi.first.wpilibj.shuffleboard.SimpleWidget;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.Joystick;

public class Collector {
    SimpleWidget collectWidget;

    WPI_TalonSRX collector_center;
    Joystick controlStick;
    DoubleSolenoid tilt1, tilt2;
    public boolean collectedAll;
    
    public Collector(Joystick _control) {
        controlStick = _control;

        collector_center = new WPI_TalonSRX(ConfigMap.COLLECTOR_MOTOR_FRONT);

        tilt1 = new DoubleSolenoid(ConfigMap.PCM_CanID, ConfigMap.CollectSolenoidChannelForward1, ConfigMap.CollectSolenoidChannelReverse1);
        tilt2 = new DoubleSolenoid(ConfigMap.PCM_CanID, ConfigMap.CollectSolenoidChannelForward2, ConfigMap.CollectSolenoidChannelReverse2);
    }

    public void intake() {
        collector_center.set(1);
    }

    public void outtake() {
        collector_center.set(-1);

    }

    public void stopMotors() {
        collector_center.set(0);
    }

    public void tiltForward() {
        tilt1.set(DoubleSolenoid.Value.kReverse);
        tilt2.set(DoubleSolenoid.Value.kReverse);
    }

    public void tiltBack() {
        tilt1.set(DoubleSolenoid.Value.kForward);
        tilt2.set(DoubleSolenoid.Value.kForward);
    }

}
