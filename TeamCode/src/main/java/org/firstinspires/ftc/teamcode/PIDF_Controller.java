package org.firstinspires.ftc.teamcode;

import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.gamepad1;
import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.hardwareMap;
import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.telemetry;

import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;

public class PIDF_Controller {

    public ElapsedTime runtime = new ElapsedTime();

    public Telemetry telemetry;

    public double tolerance;

    double targetPos;
    double currPos;

    public double previousTime = 0;
    public double error;
    public double armError;

    public double area = 0;
    public double previousArea = 0;

    public double P = 0;
    public double I = 0;
    public double D = 0;
    public double deltaTime;
    public double previousError = 0;
    public double previousArmError = 0;

    public double a = 0;
    public double armA = 0;
    public double previousFilterEstimate = 0;
    public double currentFilterEstimate = 0;
    public double previousArmFilterEstimate = 0;
    public double armCurrentFilterEstimate = 0;

    public double errorChange;
    public double armErrorChange;

    double gravity = 0;

    private double power = 0;

    public PIDF_Controller(Telemetry telemetry){
        this.telemetry = telemetry;
        this.error = tolerance + 1;
    }

    public double PIDF(double currPos, double targetPos, double kp){
        error = targetPos - currPos;

        P = kp*(error/Math.abs(targetPos)); // Proportional term : KP constant * the error of the system

        power =  P;

        telemetry.addData("Target", targetPos);
        telemetry.addData("Error", error);
        telemetry.addData("Power", power);
        telemetry.addData("Proportion:", P);

        return power;
    }

    public double PIDF(double currPos, double targetPos, double kp, double kd){

        telemetry.addData("Error", error);
        telemetry.addData("Previous Error", previousError);
        telemetry.addData("Derivative:", D);
        telemetry.addData("Power", power);
        telemetry.addData("Proportion:", P);
        telemetry.addData("Delta Time", deltaTime);
        telemetry.addData("Target", targetPos);

        error = targetPos - currPos;

        P = kp*(error/Math.abs(targetPos)); // Proportional term : KP constant * the error of the system

        deltaTime = runtime.seconds();
        runtime.reset();

        D = kd * ((error - previousError) / deltaTime);

        power =  P + D;

        previousError = error;

        return power;
    }

    public double PIDF(double currPos, double targetPos, double kp, double kd, double a){
        error = targetPos - currPos;

        P = kp*(error/Math.abs(targetPos)); // Proportional term : KP constant * the error of the system

        deltaTime = runtime.seconds();
        runtime.reset();

        errorChange = error - previousError;

        currentFilterEstimate = (1-a) * errorChange + (a * previousFilterEstimate);

        D = kd * (currentFilterEstimate/deltaTime);

        power =  P + D;

        telemetry.addData("Target", targetPos);
        telemetry.addData("Error", error);
        telemetry.addData("Previous Error", previousError);
        telemetry.addData("Power", power);
        telemetry.addData("Proportion:", P);
        telemetry.addData("Derivative:", D);
        telemetry.addData("Delta Time", deltaTime);

        previousError = error;
        previousFilterEstimate = currentFilterEstimate;

        return power;
    }

    //TODO: add "a" inside
    public double PIDF(double currPos, double targetPos, double kp, double kd, double a, double ki){
        error = targetPos - currPos;

        P = kp*(error/Math.abs(targetPos)); // Proportional term : KP constant * the error of the system

//        deltaTime = (System.currentTimeMillis() - previousTime)/1000;
//        previousTime = System.currentTimeMillis();

        deltaTime = runtime.seconds();
        runtime.reset();
//        I += Math.abs(currPos) > Math.abs(targetPos * 0.8) ? deltaTime * error : 0; //TODO: Play with this I

        area = previousArea + deltaTime * ki;

        I = area*ki;

        if ((previousError*error) < 0) area = 0;

        errorChange = error - previousError;

        currentFilterEstimate = (1-a) * errorChange + (a * previousFilterEstimate);
        previousFilterEstimate = currentFilterEstimate;

        D = kd * ((error - previousError) / deltaTime);

        power =  P + I + D;

        telemetry.addData("Target", targetPos);
        telemetry.addData("Error", error);
        telemetry.addData("Power", power);
        telemetry.addData("Proportion:", P);
        telemetry.addData("Derivative:", D);
        telemetry.addData("Integral:", I);
        telemetry.addData("Delta Time", deltaTime);
        telemetry.addData("Previous Error", previousError);

        previousError = error;
        previousArea = area;

        return power;
    }

    public double PIDF(double currPos, double targetPos, double kp, double kd, double a, double ki, double kF){
        error = targetPos - currPos;
        previousError = error;

        P = kp*(error/Math.abs(targetPos)); // Proportional term : KP constant * the error of the system

        deltaTime = runtime.seconds();
        runtime.reset();
//        I += Math.abs(currPos) > Math.abs(targetPos * 0.8) ? deltaTime * error : 0; //TODO: Play with this I

        area = previousArea + deltaTime * ki;
        previousArea = area;

        I = area*ki;

        if ((previousError*error) < 0) area = 0;

        errorChange = error - previousError;

        currentFilterEstimate = (1-a) * errorChange + (a * previousFilterEstimate);
        previousFilterEstimate = currentFilterEstimate;

        D = kd * ((error - previousError) / deltaTime);

        power =  P + I + D + kF*Math.signum(error);

        telemetry.addData("Target", targetPos);
        telemetry.addData("Error", error);
        telemetry.addData("Power", power);
        telemetry.addData("Proportion:", P);
        telemetry.addData("Derivative:", D);
        telemetry.addData("Integral:", I);
        telemetry.addData("Delta Time", deltaTime);
        telemetry.addData("Previous Error", previousError);

        return power;
    }

//    public double PIDF_Arm(double currPos, double targetPos, double kp, double kd, double ki, double a, double kGravity, double firstError){
//
//        previousArmError = armError;
//        armError = targetPos - currPos;
//
//        this.P = kp*(armError/Math.abs(firstError));
//
//        this.deltaTime = System.currentTimeMillis() - previousTime;
//        this.previousTime = System.currentTimeMillis();
//
//        area += currPos > targetPos * 0.8 ? deltaTime * armError : 0;
//        this.I = ki*area;
//
//        if (Math.abs(armError) < 2) this.I = 0;
//
//        armErrorChange = armError - previousArmError;
//
//        previousFilterEstimate = currentFilterEstimate;
//        armCurrentFilterEstimate = (1-a) * errorChange + (a * previousArmFilterEstimate);
//        this.D = kd*(armCurrentFilterEstimate / deltaTime);
//
//        gravity = kGravity*(Math.cos(Math.toRadians(currPos)));
//
//        this.power =  P + I + D + gravity;
//
//
//        return this.power;
//    }

    // TODO: gonna have to retune for "a"
    public double PIDF_Arm(double currPos, double targetPos, double kp, double kd, double ki, double a, double kGravity, double firstError){

        armError = targetPos - currPos;

        this.P = kp*(armError/Math.abs(firstError));

        this.deltaTime = System.currentTimeMillis() - previousTime;
        this.previousTime = System.currentTimeMillis();

        area += currPos > targetPos * 0.8 ? deltaTime * armError : 0;
        this.I = ki*area;

        if (Math.abs(armError) < 2) this.I = 0;

        armErrorChange = armError - previousArmError;

        previousFilterEstimate = currentFilterEstimate;
        armCurrentFilterEstimate = (1-a) * errorChange + (armA * previousArmFilterEstimate);
        this.D = kd*(armCurrentFilterEstimate / deltaTime);

        gravity = kGravity*(Math.cos(Math.toRadians(currPos)));

        this.power =  P + I + D + gravity;

        previousArmError = armError;
        return this.power;
    }


    public double getError(){
        return this.error;
    }
}
