/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package tech.team1781.infiniteRecharge;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;

import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.shuffleboard.SimpleWidget;

/**
 * Add your docs here.
 */
public class ControlPanel {
    WPI_TalonSRX controlPannelMotor;
    ShuffleboardTab controlPanelTab = Shuffleboard.getTab("Control Panel Data");
    SimpleWidget encoderCounts;
    Encoder controlPanelEncoder;
    Joystick joy;
    boolean controlPanelSpin = false;

    public ControlPanel(Joystick _joy) {
        joy = _joy;
        controlPannelMotor = new WPI_TalonSRX(ConfigMap.PANEL_MOTOR);
        encoderCounts = controlPanelTab.add("Encoder Counts", 0.0);
        controlPanelEncoder = new Encoder(ConfigMap.CONTROL_PANEL_ENCODER_DIO1, ConfigMap.CONTROL_PANEL_ENCODER_DIO2);
    }

    public void update() {
        // if (joy.getRawButton(ConfigMap.PANEL_SPIN_LEFT)) {
        //     controlPannelMotor.set(-.6);

        // } else if (joy.getRawButton(ConfigMap.PANEL_SPIN_RIGHT)) {
        //     controlPannelMotor.set(.6);

        // } else {
        //     controlPannelMotor.set(0);

        // }

        // if (joy.getRawButton(ConfigMap.COMPLETE_ROTATION_CONTROL) && !controlPanelSpin) {

        //     controlPanelEncoder.reset();
        //     controlPanelSpin = true;

        // }

        // rotationControl();

        // encoderCounts.getEntry().setDouble(controlPanelEncoder.get());
    }

    public void rotationControl() {
        if (controlPanelEncoder.get() < 30000 && controlPanelSpin) {
            controlPannelMotor.set(-.6);
        } else if (controlPanelEncoder.get() > 30000 && controlPanelSpin) {
            controlPannelMotor.set(0);
            controlPanelSpin = false;
        }
    }

}
