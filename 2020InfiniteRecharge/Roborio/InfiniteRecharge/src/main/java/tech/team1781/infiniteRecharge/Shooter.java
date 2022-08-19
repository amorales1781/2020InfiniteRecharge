/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package tech.team1781.infiniteRecharge;

import edu.wpi.first.wpilibj.Joystick;

public class Shooter {

    Collector collect;
    Turret turret;
    Conveyor conveyor;
    public boolean lockedOn, empty, collectedAll;
    Joystick pilot, coPilot;
    Camera limeLight;

    public Shooter(Joystick _pilot, Joystick _coPilot, Camera _cam) {

        pilot = _pilot;
        coPilot = _coPilot;
        limeLight = _cam;

        collect = new Collector(pilot);

        turret = new Turret(coPilot);
        conveyor = new Conveyor();
    }

    public void update() {
        turret.sendData();

        if (coPilot.getRawButton(ConfigMap.GREEN_BUTTON)) {
            turret.setShootHighSpeed(ConfigMap.GREEN_SPEED_HI);
            turret.setHoodTarget(ConfigMap.GREEN_HOOD);
        }else if (coPilot.getRawButton(ConfigMap.YELLOW_BUTTON)) {
            turret.setShootHighSpeed(ConfigMap.YELLOW_SPEED_HI);
            turret.setHoodTarget(ConfigMap.YELLOW_HOOD);
        }else if (coPilot.getRawButton(ConfigMap.BLUE_BUTTON)) {
            turret.setShootHighSpeed(ConfigMap.BLUE_SPEED_HI);
            turret.setHoodTarget(ConfigMap.BLUE_HOOD);
        }else if (coPilot.getRawButton(ConfigMap.RED_BUTTON)) {
            turret.setShootHighSpeed(ConfigMap.RED_SPEED_HI);
            turret.setHoodTarget(ConfigMap.RED_HOOD);
        }
        turret.update();

        if (pilot.getRawButton(ConfigMap.COLLECT_BUTTON))
            collect();
        else if (coPilot.getRawButton(ConfigMap.SHOOT))
            shoot();
        else if (coPilot.getRawButton(ConfigMap.REVERSE_CONVEYOR)) {
            conveyor.outtake();
        } else
            shooterIdle();

    }

    public void lockOnPort(double portTargetX) {
        limeLight.turnLightsOn();
        turret.autoAim(portTargetX);
        if (portTargetX < 1.5 && portTargetX > -1.5) {
            turret.stopTurret();
            lockedOn = true;
        } else
            lockedOn = false;
    }

    public void collect() {
        collect.tiltForward();
        collect.intake();
        conveyor.collect();
    }

    public void dontCollect() {
        collect.tiltBack();
        collect.stopMotors();
    }

    public void revShooterToSpeed() {
        turret.setShootingSpeed(2);
    }

    public void shootPowercells() {
        if (turret.shooterAtSpeed) {
            conveyor.collect();
        }
        if (turret.shootingTimer.get() >= 4) {
            empty = true;
            turret.shootingTimer.stop();
            turret.stopShooting();
        }
    }

    public void calibrate() {
        if (!turret.isCalibrated) {
            turret.aimRight(0.5);
        }
    }

    public boolean isCalibrated() {
        return turret.isCalibrated();
    }

    public void stopUpperConvey() {
        conveyor.upperStop();
    }

    public void tiltCollectorForward() {
        collect.tiltForward();
    }

    public void startTimer() {
        turret.shootingTimer.start();
    }

    public void resetTimer() {
        turret.shootingTimer.reset();
    }

    public void rotateTurretToAngle() {
        turret.rotateToAngle(180);
    }

    public void shootAtRPM() {
        turret.setShootingSpeed(-250000);
    }

    public void idle() {
        turret.stopShooting();
        conveyor.stopMotors();
        collect.stopMotors();
    }

    public void preAutoShoot() {
        turret.shoot();
        lockOnPort(limeLight.getPortX());
    }

    public void autoShoot() {
        lockOnPort(limeLight.getPortX());
        collect.intake();
        conveyor.collect();
        collect.tiltForward();
        turret.shoot();
    }

    public void autoCollect() {
        conveyor.stopMotors();
        collect.intake();

        collect.tiltForward();
        turret.shoot();
    }

    public void shoot() {
        System.out.println("Shooting Speed: " + turret.getSpeed()+", " + turret.hoodPosition.getAverageVoltage());
        turret.shoot();
        conveyor.align();
        if (turret.atSpeed()) {
            conveyor.feed();
        } else {
            conveyor.stopMotors();
        }
    }

    public void shooterIdle() {
        collect.tiltBack();
        turret.stopShooting();
        collect.stopMotors();
        conveyor.stopMotors();
    }

}
