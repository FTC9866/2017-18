package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.robotcore.external.matrices.VectorF;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.navigation.RelicRecoveryVuMark;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackableDefaultListener;


@TeleOp(name="BlueAutonomous2", group="TeleOp")

public class BlueAutonomous2 extends VirusMethods {
    enum state  {dropArm,scanJewel,knockJewelRight, knockJewelLeft, stop, goToPosition, debug, alignStraight, toCryptoBox, backOnStone, faceCryptoBox, placeGlyph, turnBackLeft, turnBackRight, turnBack, toCryptoBoxpart1, toCryptoBoxpart2, turn90, moveUntilScanned}
    state state;
    boolean setMotor;
    boolean knock;

    public void init() {
        super.init();
        cube1.setPosition(0);
        cube2.setPosition(1);
        cube3.setPosition(0);
        cube4.setPosition(1);
        lift.setPosition(0);
        jewelKnocker.setPosition(0);
        jewelKnocker.setPosition(0);
        cube1.setPosition(.5);
        cube2.setPosition(.5);
    }

    public void start() {
        lmotor0.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        lmotor1.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rmotor0.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rmotor1.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        cube3.setPosition(.15);
        cube4.setPosition(.85);
        topGrabberClose();
        state=state.dropArm;
        vuforiaInit();
    }
    @Override

    public void loop() {
        vuMark = RelicRecoveryVuMark.from(relicTemplate);
        pose = ((VuforiaTrackableDefaultListener)relicTemplate.getListener()).getPose();
        if (pose != null) {
            VectorF trans = pose.getTranslation();
            Orientation rot = Orientation.getOrientation(pose, AxesReference.EXTRINSIC, AxesOrder.XYZ, AngleUnit.DEGREES);

            // Extract the X, Y, and Z components of the offset of the target relative to the robot
            double tX = trans.get(0);
            double tY = trans.get(1);
            double tZ = trans.get(2);

            // Extract the rotational components of the target relative to the robot
            double rX = rot.firstAngle;
            double rY = rot.secondAngle;
            double rZ = rot.thirdAngle;
        }
        switch (state) {
            case dropArm:

                jewelKnocker.setPosition(0.65);
                colorSensor.enableLed(true);
                waitTime(1000);
                resetEncoder();
                state=state.scanJewel;
                break;

            case scanJewel:
                if (colorSensor.red() < colorSensor.blue()) { //checks to see if object is more red or more blue
                    knock  = true;
                    colorSensor.enableLed(false);
                    state=state.knockJewelLeft;
                }
                else if (colorSensor.blue() < colorSensor.red()) {
                    knock = false;
                    state=state.knockJewelRight;
                }
                break;

            case knockJewelLeft:
                if (turn(350, 0.5)){
                    jewelKnocker.setPosition(0);
                    state=state.turnBack;
                }
                break;

            case knockJewelRight:
                if (turn(10, 0.5)) {
                    jewelKnocker.setPosition(0);
                    state = state.turnBack;
                }
                break;
            case turnBack:
                if (turn(0, 0.5)){
                    position = lmotor0.getCurrentPosition();
                    state = state.moveUntilScanned;
                }
                break;
            //turnBackLeft and turnBackRight kept just in case turnMotorsPlus method doesn't work
            case turnBackLeft:
                turnMotors(0, true, 0.3);
                state = state.moveUntilScanned;
                break;

            case turnBackRight:
                turnMotors(0, false, 0.3);
                state = state.moveUntilScanned;
                break;

            case moveUntilScanned:
                runMotors(.1,.1,.1,.1); //program to make it move backwards if it doesn't see it after traveling a certain distance
                    if(vuMark != RelicRecoveryVuMark.UNKNOWN){
                        telemetry.addData("VuMark", "%s visible", vuMark);
                        VuMarkStored = vuMark;
                        amountMovedForward = (lmotor0.getCurrentPosition()-position)*inPerPulse; //how many inches it moved back to scan the vision target
                        state = state.alignStraight;
                    }
                break;
            case alignStraight:
                if (turn(0,1)) {
                    resetEncoder();
                    counter = 0;
                    state = state.toCryptoBoxpart1;
                }
                break;
            case backOnStone: // broken plz fix
                if (setMotorPositions(0,0,0,0, .5)){
                    resetEncoder();
                    state=state.toCryptoBox;
                }
                break;
            case toCryptoBoxpart1:
                if (setMotorPositionsINCH(-27-amountMovedForward,-27-amountMovedForward,-27-amountMovedForward,-27-amountMovedForward,-.5)){
                    resetEncoder();
                    state = state.turn90;
                }
                break;
            case turn90:
                if (turn(90,0.5)){
                    resetEncoder();
                    state = state.toCryptoBoxpart2;
                }
                break;
            case toCryptoBoxpart2:
                lift(0.03); //so that cube doesn't drag on ground
                //change the motor position values as needed after testing on field
                if (VuMarkStored == RelicRecoveryVuMark.LEFT){
                    if (setMotorPositionsINCH(-4,-4,-4,-4, -.5)){
                        resetEncoder();
                        telemetry.addData("reee", "e");
                        state=state.faceCryptoBox;
                    }
                }
                if (VuMarkStored == RelicRecoveryVuMark.CENTER){
                    if (setMotorPositionsINCH(-12,-12,-12,-12, -.5)){
                        resetEncoder();
                        state=state.faceCryptoBox;
                    }
                }
                if (VuMarkStored == RelicRecoveryVuMark.RIGHT){
                    if (setMotorPositionsINCH(-20,-20,-20,-20, .5)){
                        resetEncoder();
                        state=state.faceCryptoBox;
                    }
                }else { //just in case of some weird circumstance that it forgets the VuMark
                    if (setMotorPositionsINCH(-12,-12,-12,-12,.5)){ //parks in safe zone in front of cryptobox
                        resetEncoder();
                        state = state.stop;
                    }
                }
                break;
            case faceCryptoBox:
                if (turn(180,.75)) {
                    resetEncoder();
                    state=state.placeGlyph;
                }
                break;
            case placeGlyph:
                runMotors(0.5,0.5,0.5,0.5);
                waitTime(500);
                runMotors(0,0,0,0);
                topGrabberOpen();
                runMotors(-0.5,-0.5,-0.5,-0.5);
                waitTime(400);
                runMotors(0,0,0,0);
                state = state.stop;
                break;
            case debug:
                //telemetry.addData("done","done");
                telemetry.addData("setMotor returns", setMotor);
                telemetry.addData("inPerPulse", inPerPulse);
                telemetry.addData("left motor", lmotor0.isBusy());
                telemetry.addData("counter",counter);
                break;

            case stop:
                telemetry.addData("state", state);
                runMotors(0,0,0,0);
                break;
        }
        telemetry.addData("Blue: true Red: false ", knock);
        telemetry.addData("state", state);
        // Telemetry();,k
    }
}