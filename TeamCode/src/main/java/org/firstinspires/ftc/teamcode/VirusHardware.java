package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.GyroSensor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.matrices.OpenGLMatrix;
import org.firstinspires.ftc.robotcore.external.navigation.RelicRecoveryVuMark;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackable;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackables;

/**
 * Created by mzhang on 11/12/2017.
 */

public abstract class VirusHardware extends OpMode {
    DcMotor rmotor0; //0 is the front
    DcMotor lmotor0;
    DcMotor rmotor1;
    DcMotor lmotor1;
    final double inPerPulse=.0175; //experimentally determined value
    Servo jewelKnocker;
    ColorSensor colorSensor;
    Servo cube1;
    Servo cube2;
    Servo cube3;
    Servo cube4;
    Servo lift;
    GyroSensor gyroSensor;
    double maxPower=1;
    double lefty;
    double leftx;
    double righty;
    double rightx;
    double rtrigger;
    double ltrigger;
    double var1;
    double var2;
    public static final String TAG = "Vuforia VuMark Sample";
    OpenGLMatrix lastLocation = null;
    VuforiaLocalizer vuforia;
    int cameraMonitorViewId;
    VuforiaTrackables relicTrackables;
    VuforiaTrackable relicTemplate;
    RelicRecoveryVuMark vuMark;
    OpenGLMatrix pose;
    RelicRecoveryVuMark VuMarkStored;
    public ElapsedTime mRunTime = new ElapsedTime();

    public void init(){
        lmotor0 = hardwareMap.dcMotor.get("lmotor0");
        rmotor0 = hardwareMap.dcMotor.get("rmotor0");
        lmotor1 = hardwareMap.dcMotor.get("lmotor1");
        rmotor1 = hardwareMap.dcMotor.get("rmotor1");
        lmotor0.setDirection(DcMotor.Direction.REVERSE);
        lmotor1.setDirection(DcMotor.Direction.REVERSE);
        lmotor0.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        lmotor1.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rmotor0.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rmotor1.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        cube1 = hardwareMap.servo.get("cube1");
        cube2 = hardwareMap.servo.get("cube2");
        cube3 = hardwareMap.servo.get("cube3");
        cube4 = hardwareMap.servo.get("cube4");
        lift = hardwareMap.servo.get("lift");
        jewelKnocker = hardwareMap.servo.get("jewelKnocker");
        colorSensor = hardwareMap.colorSensor.get("colorSensor");
        gyroSensor = hardwareMap.gyroSensor.get("gryoSensor");
        while (gyroSensor.isCalibrating());


    }



}
