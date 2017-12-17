package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.matrices.OpenGLMatrix;
import org.firstinspires.ftc.robotcore.external.matrices.VectorF;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.navigation.RelicRecoveryVuMark;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;

public abstract class VirusMethods extends VirusHardware{
    int counter=0;
    double position;
    double amountMovedForward;
    double turnRate;
    double angleRel;
    double maxDisplacement;
    boolean triggered;
    int cryptoboxSection;
    String[][]cryptobox = {{"brown","gray","gray"},{"brown","brown","gray"},{"gray","brown","brown"},{"gray","gray","brown"}};
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
        lmotor0.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        lmotor1.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rmotor0.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rmotor1.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
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
    public void runMotorsAuto(double Left0, double Left1, double Right0, double Right1){
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
            runMotorsAuto(power, power, power, power);
            counter++;
        }
        return !lmotor0.isBusy() && !lmotor1.isBusy() && !rmotor0.isBusy() && !rmotor1.isBusy(); //returns true when motors are not busy
    }

    public boolean setMotorPositionsINCH(double Left0, double Left1, double Right0, double Right1, double power){
        lmotor0.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        lmotor1.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rmotor0.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rmotor1.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        if (counter == 0){ //makes sure this is only run once, reset back to 0 when OpMode starts or resetEncoders is called
            lmotor0.setTargetPosition((int)(Left0/inPerPulse));
            lmotor1.setTargetPosition((int)(Left1/inPerPulse));
            rmotor0.setTargetPosition((int)(Right0/inPerPulse));
            rmotor1.setTargetPosition((int)(Right1/inPerPulse));
            runMotorsAuto(power,power,power,power);
            counter++;
        }
        return (!lmotor0.isBusy() && !lmotor1.isBusy() && !rmotor0.isBusy() && !rmotor1.isBusy()); //returns true when motors are not busy
    }

//    public boolean turn (double angle, double speed) {
//        double threshold = 1;
//        turnRate=(speed*angleDistance(angle, gyroSensor.getHeading())/90); //preferably, 90 is changed to the initial distance from the angle. I couldn't find a good way for that to work
//        if (turnRate>0){
//            turnRate+=.05; //you guys might want to experiment with this value
//        }
//        if (turnRate<0){
//            turnRate-=.05; //you guys might want to experiment with this value
//        }
//
//        runMotors(turnRate, turnRate, -turnRate, -turnRate);
//        if (-threshold< angleDistance(angle, gyroSensor.getHeading())&& angleDistance(angle, gyroSensor.getHeading())<threshold)
//        {
//            return true;
//        }
//        return false;
//    }

    private double angleDistance(double angle, double currentAngle) {
        double distance= angle - currentAngle;
        if (angle < -180){
            distance+=180;
        }
        return distance;
    }

    public boolean turnMotors(double angle, boolean right, double speed) {
        turnRate=(speed*absoluteDistance(angle,gyroSensor.getHeading())/90);
        if (right) {
            runMotors(-turnRate, -turnRate, turnRate, turnRate);
        } else {
            runMotors(turnRate, turnRate, -turnRate, -turnRate);
        }
        telemetry.addData("distance left:", absoluteDistance(angle, gyroSensor.getHeading()));

        if (absoluteDistance(angle, gyroSensor.getHeading()) < 15) {

            return true;
        }
        return false;
    }

    private double absoluteDistance(double angle1, double angle2) {

        double angleDistance = Math.abs(angle1 - angle2);

        if (angleDistance > 180) {
            angleDistance = 360 - angleDistance;
        }
        return angleDistance;
    }
    public boolean turn(double angle, double speed){
        angle=360-angle;
        double threshold = 1;
        double currentAngle = gyroSensor.getHeading();
        angleRel = relativeAngle(angle, currentAngle); //should be distance from current angle (negative if to the counterclockwise, positive if to the clockwise)
        turnRate = speed*angleRel/90;
        runMotors(turnRate, turnRate, -turnRate, -turnRate); //negative turnRate will result in a left turn
        if (angleRel<=threshold && angleRel>=-threshold) { //approaching from either side
            return true;
        }
        return false;
    }
    private double relativeAngle(double angle, double currentAngle){
        double currentAngleRel = angle-currentAngle;
        if (currentAngleRel > 180){
            currentAngleRel -= 360;
        }else if (currentAngle < -179){
            currentAngleRel += 360;
        }
        return currentAngleRel;
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
    public void vuforiaInit(){
        VuforiaLocalizer.Parameters parameters = new VuforiaLocalizer.Parameters(cameraMonitorViewId);
        parameters.vuforiaLicenseKey = "AQmuIUP/////AAAAGR6dNDzwEU07h7tcmZJ6YVoz5iaF8njoWsXQT5HnCiI/oFwiFmt4HHTLtLcEhHCU5ynokJgYSvbI32dfC2rOvqmw81MMzknAwxKxMitf8moiK62jdqxNGADODm/SUvu5a5XrAnzc7seCtD2/d5bAIv1ZuseHcK+oInFHZTi+3BvhbUyYNvnVb0tQEAv8oimzjiQW18dSUcEcB/d6QNGDvaDHpxuRCJXt8U3ShJfBWWQEex0Vp6rrb011z8KxU+dRMvGjaIy+P2p5GbWXGJn/yJS9oxuwDn3zU6kcQoAwI7mUgAw5zBGxxM+P35DoDqiOja6ST6HzDszHxClBm2dvTRP7C4DEj0gPkhX3LtBgdolt";
        parameters.cameraDirection = VuforiaLocalizer.CameraDirection.FRONT;
        this.vuforia = ClassFactory.createVuforiaLocalizer(parameters);
        cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        relicTrackables = this.vuforia.loadTrackablesFromAsset("RelicVuMark");
        relicTemplate = relicTrackables.get(0);
        relicTemplate.setName("relicVuMarkTemplate"); // can help in debugging; otherwise not necessary
        relicTrackables.activate();
    }
    public void lift(double position) {
        lift.setPosition(position);
    }
    public void topGrabberOpen(){
        cube3.setPosition(.25);
        cube4.setPosition(.75);
    }
    public void topGrabberClose(){
        cube3.setPosition(.6);
        cube4.setPosition(.4);
    }
    public String GPS(boolean bottom){
        int bottomx;
        int bottomy;
        if (cryptoboxSection==0){
            bottomx = 0;
            bottomy = 3;
        }else if (cryptoboxSection==1){
            bottomx = 0;
            bottomy = 1;
        }else if (cryptoboxSection==2){
            bottomx = 1;
            bottomy = 3;
        }else if (cryptoboxSection==3){
            bottomx = 1;
            bottomy = 1;
        }else if (cryptoboxSection==4){
            bottomx = 2;
            bottomy = 3;
        }else{
            bottomx = 2;
            bottomy = 1;
        }
        if (bottom){
            return cryptobox[bottomy][bottomx]; //returns block for bottom grabber
        }else{
            return cryptobox[bottomy-1][bottomx]; //returns block for top grabber
        }
    }
    public String cryptoboxLocation(){
        if (cryptoboxSection==0 || cryptoboxSection==1){
            return "Left";
        }else if (cryptoboxSection==2||cryptoboxSection==3){
            return "Center";
        }else{
            return "Right";
        }
    }
    String format(OpenGLMatrix transformationMatrix) {
        return (transformationMatrix != null) ? transformationMatrix.formatAsTransform() : "null";
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
