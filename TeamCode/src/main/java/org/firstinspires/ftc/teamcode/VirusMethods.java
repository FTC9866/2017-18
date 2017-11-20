package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.Range;

public abstract class VirusMethods extends VirusHardware{
    int counter=0;
    double turnRate;
    public void runMotors(double Left0, double Left1, double Right0, double Right1, double steerMagnitude){
        if (Left0!=0&&Left1!=0&&Right0!=0&&Right1!=0) {
            steerMagnitude *= 2 * Math.max(Math.max(Left0, Left1), Math.max(Right0, Right1));
        }
        Left0=Left0+steerMagnitude;
        Left1=Left1+steerMagnitude;
        Right0=Right0-steerMagnitude;
        Right1=Right1-steerMagnitude;
        //make sure no exception thrown if power > 0
        Left0 = Range.clip(Left0, -maxPower, maxPower);
        Left1 = Range.clip(Left1, -maxPower, maxPower);
        Right0 = Range.clip(Right0, -maxPower, maxPower);
        Right1 = Range.clip(Right1, -maxPower, maxPower);
        rmotor0.setPower(Right0);
        rmotor1.setPower(Right1);
        lmotor0.setPower(Left0);
        lmotor1.setPower(Left1);
    }

    public void runMotors(double Left0, double Left1, double Right0, double Right1){
        //make sure no exception thrown if power > 0
        Left0 = Range.clip(Left0, -maxPower, maxPower);
        Left1 = Range.clip(Left1, -maxPower, maxPower);
        Right0 = Range.clip(Right0, -maxPower, maxPower);
        Right1 = Range.clip(Right1, -maxPower, maxPower);
        rmotor0.setPower(Right0);
        rmotor1.setPower(Right1);
        lmotor0.setPower(Left0);
        lmotor1.setPower(Left1);
    }

    public boolean setMotorPositions(int Left0, int Left1, int Right0, int Right1, double power) {
        if (counter == 0) { //makes sure this is only run once, reset back to 0 when OpMode starts or resetEncoders is called
            lmotor0.setTargetPosition(Left0);
            lmotor1.setTargetPosition(Left1);
            rmotor0.setTargetPosition(Right0);
            rmotor1.setTargetPosition(Right1);
            runMotors(power, power, power, power);
            counter++;
        }
        return !lmotor0.isBusy() && !lmotor1.isBusy() && !rmotor0.isBusy() && !rmotor1.isBusy(); //returns true when motors are not busy
    }

    public boolean setMotorPositionsINCH(double Left0, double Left1, double Right0, double Right1, double power){
        if (counter == 0){ //makes sure this is only run once, reset back to 0 when OpMode starts or resetEncoders is called
            lmotor0.setTargetPosition((int)(Left0/inPerPulse));
            lmotor1.setTargetPosition((int)(Left1/inPerPulse));
            rmotor0.setTargetPosition((int)(Right0/inPerPulse));
            rmotor1.setTargetPosition((int)(Right1/inPerPulse));
            runMotors(power,power,power,power);
            counter++;
        }
        return (!lmotor0.isBusy() && !lmotor1.isBusy() && !rmotor0.isBusy() && !rmotor1.isBusy()); //returns true when motors are not busy
    }

    public boolean turn (double angle, double speed){
        turnRate=speed/angle*Math.abs(gyroSensor.getHeading()-angle);
        if (angle<=180) {
            runMotors(turnRate, turnRate, -turnRate, -turnRate);
            if (gyroSensor.getHeading()>=angle){
                return true;
            }
        }//speed is always positive
        if (angle>180){
            runMotors(-turnRate, -turnRate, turnRate, turnRate);
            if (gyroSensor.getHeading()<=angle){
                return true;
            }
        }
        return false;
    }

    public void resetEncoder(){
        runMotors(0,0,0,0);
        lmotor0.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        lmotor1.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rmotor0.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rmotor1.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        counter=0; // sets counter = 0 for setMotorPosition method
        while (lmotor0.isBusy()||lmotor1.isBusy()||rmotor0.isBusy()||rmotor1.isBusy()); //waits until encoders finish reset
        lmotor0.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        lmotor1.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rmotor0.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rmotor1.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    }

    public void waitTime(int time){
        try {
            Thread.sleep(time);
        }
        catch (InterruptedException e) {
            e.printStackTrace();
        }
    }

    public void updateControllerValues(){
        lefty = -gamepad1.left_stick_y;
        leftx = -gamepad1.left_stick_x;
        righty = -gamepad1.right_stick_y;
        rightx = -gamepad1.right_stick_x;
        rtrigger = -gamepad1.right_trigger;
        ltrigger = -gamepad1.left_trigger;
        double scalar = Math.max(Math.abs(lefty-leftx), Math.abs(lefty+leftx)); //scalar and magnitude scale the motor powers based on distance from joystick origin
        double magnitude = Math.sqrt(lefty*lefty+leftx*leftx);
        var1= (lefty-leftx)*magnitude/scalar;
        var2= (lefty+leftx)*magnitude/scalar;
    }
    public void Telemetry(){
        telemetry.addData("Red",colorSensor.red());
        telemetry.addData("Green",colorSensor.green());
        telemetry.addData("Blue",colorSensor.blue());

        telemetry.addData("lMotor0 Encoder",lmotor0.getCurrentPosition());
        telemetry.addData("lMotor1 Encoder",lmotor1.getCurrentPosition());
        telemetry.addData("rMotor0 Encoder",rmotor0.getCurrentPosition());
        telemetry.addData("rMotor1 Encoder",rmotor1.getCurrentPosition());

        telemetry.addData("lMotor0 Target",lmotor0.getTargetPosition());
        telemetry.addData("lMotor1 Target",lmotor1.getTargetPosition());
        telemetry.addData("rMotor0 Target",rmotor0.getTargetPosition());
        telemetry.addData("rMotor1 Target",rmotor1.getTargetPosition());



    }
}
