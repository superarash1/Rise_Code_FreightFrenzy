package org.firstinspires.ftc.teamcode.Hardware.BotMechanisms;

import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.ServoImplEx;

public class Cage {
    public ServoImplEx cageSpin;

    private Gamepad gamepad1;

    public double boxTarget = 0.7;

    public enum cageState {
        DROP,
        FLAT,
        MANUAL
    }

    cageState CageState;

    public Cage (String name, HardwareMap hardwareMap, Gamepad gamepad1){
        cageSpin = hardwareMap.get(ServoImplEx.class, name);

        this.gamepad1 = gamepad1;

        CageState = cageState.FLAT;
    }

    public void cageRotation(){
        cageSpin.setPosition(boxTarget);

        switch (CageState){
            case FLAT:
                boxTarget = 0;

                break;
            case DROP:
                boxTarget = 0.7;

                break;
            case MANUAL:
//                if (gamepad1.right_bumper) {
//                    cageSpin.setPosition(boxTarget - 0.01);
//                } else if (gamepad1.left_bumper) {
//                    cageSpin.setPosition(boxTarget + 0.01);
//                }

                if (gamepad1.right_bumper) {
                    boxTarget = boxTarget - 0.01;
                } else if (gamepad1.left_bumper) {
                    boxTarget = boxTarget - 0.01;;
                }

                break;
        }
    }
}
