package org.firstinspires.ftc.teamcode;

import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.gamepad1;
import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.hardwareMap;
import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.telemetry;

import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;

public class PIDF_Controller {

    private ElapsedTime runtime = new ElapsedTime();

    private double previousTime = 0;
    private double error;


    private double area = 0;
    private double previousArea = 0;

    private double P = 0;
    private double I = 0;
    private double D = 0;
    private double deltaTime;
    private double previousError = 0;

    private double a = 0;
    private double previousFilterEstimate = 0;
    private double currentFilterEstimate = 0;
    private double errorChange;

    double gravity = 0;

    private double power = 0;

    public PIDF_Controller(){

    }

    public double PIDF_Drive(double currPos, double targetPos, double kp, double kd, double ki, double kStatic, double tolerance){
        error = targetPos - currPos;
        previousError = error;
        P = Math.abs(error)/targetPos; // Proportional term : KP constant * the error of the system

        this.previousTime = System.currentTimeMillis();
        this.deltaTime = System.currentTimeMillis() - previousTime;

        I += currPos > targetPos * 0.8 ? deltaTime * error : 0; //TODO: Play with this I

        errorChange = error - previousError;

        previousFilterEstimate = currentFilterEstimate;
        currentFilterEstimate = (1-a) * errorChange + (a * previousFilterEstimate);
        this.D = (error - previousError) / deltaTime;

        this.power =  kp*P + ki*I + kd*D + kStatic * Math.signum(error);

        return this.power;

    }

    public double PIDF_Strafe(double currPos, double targetPos, double kp, double kd, double ki, double kStatic, double tolerance){
        error = targetPos - currPos;
        previousError = error;
        P = Math.abs(error)/targetPos;

        this.previousTime = System.currentTimeMillis();
        this.deltaTime = System.currentTimeMillis() - previousTime;

        I += currPos > targetPos * 0.8 ? deltaTime * error : 0; //TODO: Play with this I

        errorChange = error - previousError;

        previousFilterEstimate = currentFilterEstimate;
        currentFilterEstimate = (1-a) * errorChange + (a * previousFilterEstimate);
        this.D = (error - previousError) / deltaTime;

        this.power =  kp*P + ki*I + kd*D + kStatic * Math.signum(error);

        return this.power;
    }

    public double PIDF_Turn(double currPos, double targetPos, double kp, double kd, double ki, double kStatic, double tolerance){
        error = targetPos - currPos;
        previousError = error;
        P = Math.abs(error)/targetPos;

        this.deltaTime = runtime.seconds();


        I += currPos > targetPos * 0.8 ? deltaTime * error : 0; //TODO: Play with this I

        errorChange = error - previousError;

        previousFilterEstimate = currentFilterEstimate;
        currentFilterEstimate = (1-a) * errorChange + (a * previousFilterEstimate);
        this.D = currentFilterEstimate / deltaTime;

        this.power =  kp*P + ki*I + kd*D + kStatic * Math.signum(error);


        return this.power;
    }

    public double PIDF_Arm(double currPos, double targetPos, double kp, double kd, double ki, double a, double kGravity, double firstError){

        previousError = error;
        error = targetPos - currPos;

        this.P = kp*(error/Math.abs(firstError));

        this.deltaTime = System.currentTimeMillis() - previousTime;
        this.previousTime = System.currentTimeMillis();

        area += currPos > targetPos * 0.8 ? deltaTime * error : 0;
        this.I = ki*area;

        if (Math.abs(error) < 2) this.I = 0;

        errorChange = error - previousError;

        previousFilterEstimate = currentFilterEstimate;
        currentFilterEstimate = (1-a) * errorChange + (a * previousFilterEstimate);
        this.D = kd*(currentFilterEstimate / deltaTime);

        gravity = kGravity*(Math.cos(Math.toRadians(currPos)));

        this.power =  P + I + D + gravity;

        return this.power;
    }

    public double getError(){
        return this.error;
    }

    public double getProportion(){
        return this.P;
    }
    public double getDerivative(){
        return this.D;
    }
    public double getIntegral(){
        return this.I;
    }
    public double getGravity(){
        return this.gravity;
    }
    public double getPower(){
        return this.power;
    }
    public double getDeltaTime(){
        return this.deltaTime;
    }
}
