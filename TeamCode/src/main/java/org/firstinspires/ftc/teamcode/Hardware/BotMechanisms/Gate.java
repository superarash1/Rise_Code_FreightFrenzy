package org.firstinspires.ftc.teamcode.Hardware.BotMechanisms;

import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

public class Gate {

    Servo gate;

    public enum gateState {
        OPEN,
        CLOSED
    }

    public gateState GateState;

    public Gate(String name, HardwareMap hardwareMap){
        gate = hardwareMap.get(Servo.class, name); // name is "gate"

        GateState = gateState.CLOSED;
    }

    public void toggleGate(){

        /* TODO: Might need to switch open and closed positions cuz I don't remember which one is which
            (including initial state in constructor)
         */
        switch (GateState) {
            case OPEN:
                gate.setPosition(0.6);
                break;
            case CLOSED:
                gate.setPosition(0);
                break;
        }
    }

    public void setGatePosition(double target){
        gate.setPosition(target);
    }
}
