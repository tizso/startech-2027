package org.firstinspires.ftc.teamcode;

import static com.qualcomm.robotcore.util.ElapsedTime.Resolution.SECONDS;

import com.qualcomm.hardware.rev.Rev2mDistanceSensor;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

public class HardwareBox extends LinearOpMode{

    double SLIDER_DIFF = 0.9435;
    //public NormalizedColorSensor color;

    //public DistanceSensor sensorDistance;

    //public DcMotor intake = null;
   // public DcMotorEx outtakeLeft = null;
   // public DcMotorEx outtakeRight = null;

    //public CRServo servoInR = null;
    //public CRServo servoInL = null;

    HardwareMap hwMap =  null;
    private ElapsedTime period  = new ElapsedTime();

    /* Constructor */
    public HardwareBox(){

    }
    public void runOpMode(){}

    /* Initialize standard Hardware interfaces */
    public void init(HardwareMap ahwMap) {
        // Save reference to Hardware map
        hwMap = ahwMap;

        // Define and Initialize Motors
        /*intake = hwMap.get(DcMotor.class, "intake");
        outtakeLeft = hwMap.get(DcMotorEx.class, "outtakeLeft");
        outtakeRight = hwMap.get(DcMotorEx.class, "outtakeRight");

        intake.setDirection(DcMotorSimple.Direction.REVERSE);
        outtakeLeft.setDirection(DcMotorEx.Direction.FORWARD);
        outtakeRight.setDirection(DcMotorEx.Direction.FORWARD);

        intake.setPower(0);
        intake.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        intake.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        outtakeLeft.setPower(0);
        outtakeLeft.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        outtakeLeft.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);

        outtakeRight.setPower(0);
        outtakeRight.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        outtakeRight.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);

        //color = hardwareMap.get(NormalizedColorSensor.class, "color");

        // Define and initialize ALL installed servos.

        servoInR = hwMap.get(CRServo.class, "servoInR");
        servoInL = hwMap.get(CRServo.class, "servoInL");

        servoInR.setDirection(DcMotorSimple.Direction.REVERSE);*/
    }

    /*public void safeWaitSeconds(double time) {
        ElapsedTime timer = new ElapsedTime(SECONDS);
        timer.reset();
        while (!isStopRequested() && timer.time() < time) {
        }
    }*/


    /*public void setIntake(double value){
        intake.setPower(value);
    }*/

}
