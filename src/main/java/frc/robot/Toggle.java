package frc.robot;

import edu.wpi.first.wpilibj.Joystick;

public class Toggle {

    public void updateToggle(Joystick joystick, byte button, boolean toggleOn, boolean togglePressed) {

        if(joystick.getRawButton(button)) {
            if(!togglePressed) {
                toggleOn = !toggleOn;
                togglePressed = true;
            }
        } else {
            togglePressed = false;
        }

    }

}