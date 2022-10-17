package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorController;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DistanceSensor;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

public class RobotClass {
    private LinearOpMode myOpMode = null;

    // Motors
    public DcMotor FLDrive;
    public DcMotor FRDrive;
    public DcMotor BLDrive;
    public DcMotor BRDrive;

    HardwareMap hwMap = null;
    public ElapsedTime timeElapsed = new ElapsedTime();

    public RobotClass(HardwareMap hwMap) {
        init(hwMap);
    }

    /* Initialize standard Hardware interfaces */
    public void init(HardwareMap hwMap) {
        // Save reference to Hardware map
        hwMap = hwMap;

        // Define and initialize motors
        FLDrive = hwMap.get(DcMotor.class, "FLDrive");
        FRDrive = hwMap.get(DcMotor.class, "FRDrive");
        BLDrive = hwMap.get(DcMotor.class, "BLDrive");
        BRDrive = hwMap.get(DcMotor.class, "BRDrive");
        // ? ServoPlaceholder = hwMap.get(CRServo.class, "ServoPlaceholder");

        // Make robot drive straight
        FLDrive.setDirection(DcMotor.Direction.REVERSE);
        BLDrive.setDirection(DcMotor.Direction.REVERSE);

        FLDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        FRDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        BLDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        BRDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        FLDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        FRDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        BLDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        BRDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        FLDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        BLDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        FRDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        BRDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }

    public void resetDrive() {
        FLDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        FRDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        BLDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        BRDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        FLDrive.setTargetPosition(0);
        FRDrive.setTargetPosition(0);
        BLDrive.setTargetPosition(0);
        BRDrive.setTargetPosition(0);
    }

    public void runToPosDrive() {
        // Runs to position set by setTargetPosition
        FLDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        FRDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        BLDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        BRDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    }

    public void stopDrive() {
        FLDrive.setPower(0);
        FRDrive.setPower(0);
        BLDrive.setPower(0);
        BRDrive.setPower(0);
    }

    public void setDrivePower(double FL, double FR, double BL, double BR) {
        FLDrive.setPower(FL);
        FRDrive.setPower(FR);
        BLDrive.setPower(BL);
        BRDrive.setPower(BR);
    }

}
