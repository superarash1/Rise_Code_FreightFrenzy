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

    double kp;
    double kd;
    double ki;
    double a;
    double kF;

    public double previousTime = 0;
    public double error;
    public double armError;

    public double area;
    public double previousArea = 0;

    public double P = 0;
    public double I = 0;
    public double D = 0;

    public double deltaTime;
    public double previousError = 0;
    public double previousArmError = 0;

    public double armA = 0;
    public double previousFilterEstimate = 0;
    public double currentFilterEstimate = 0;
    public double previousArmFilterEstimate = 0;
    public double armCurrentFilterEstimate = 0;

    public double errorChange;
    public double armErrorChange;

    double gravity = 0;

    private double power;

    //TODO: ILQR

    public PIDF_Controller(double kp, Telemetry telemetry){
        this.kp = kp;

        a = 0;
        kd = 0;
        ki = 0;
        kF = 0;

        this.telemetry = telemetry;
        this.error = tolerance + 1;
    }

    public PIDF_Controller(double kp, double kd, Telemetry telemetry){
        this.kp = kp;
        this.kd = kd;

        a = 0;
        ki = 0;
        kF = 0;

        this.telemetry = telemetry;
        this.error = tolerance + 1;
    }

    public PIDF_Controller(double kp, double kd, double a, Telemetry telemetry){
        this.kp = kp;
        this.kd = kd;
        this.a = a;

        ki = 0;
        kF = 0;

        this.telemetry = telemetry;
        this.error = tolerance + 1;
    }

    public PIDF_Controller(double kp, double kd, double a, double ki, Telemetry telemetry){
        this.kp = kp;
        this.kd = kd;
        this.a = a;
        this.ki = ki;

        kF = 0;

        this.telemetry = telemetry;
        this.error = tolerance + 1;
    }

    public PIDF_Controller(double kp, double kd, double a, double ki, double kF, Telemetry telemetry){
        this.kp = kp;
        this.kd = kd;
        this.a = a;
        this.ki = ki;
        this.kF = kF;

        this.telemetry = telemetry;
        this.error = tolerance + 1;
    }

    public void PIDF(double currPos, double targetPos){
        error = targetPos - currPos;

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

//        D = kd * (currentFilterEstimate / deltaTime);
        D = kd * (error - previousError / deltaTime);

        power = P + I + D + kF*Math.signum(error);

        telemetry.addData("Target", targetPos);
        telemetry.addData("Error", error);
        telemetry.addData("Previous Error", previousError);
        telemetry.addData("Power", power);
        telemetry.addData("Proportion:", P);
        telemetry.addData("Derivative:", D);
        telemetry.addData("Integral:", I);
        telemetry.addData("Delta Time", deltaTime);

        previousError = error;
    }

    public double PIDF_Power(){
        return P + I + D + kF*Math.signum(error);
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
    // TODO: Separate into separate class
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
