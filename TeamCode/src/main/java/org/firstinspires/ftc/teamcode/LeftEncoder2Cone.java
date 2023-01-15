package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;

// import org.openftc.easyopencv.OpenCvCamera;
// import org.openftc.easyopencv.OpenCvCameraFactory;
// import org.openftc.easyopencv.OpenCvCameraRotation;

@Autonomous(name = "leftencode crazyyyy", group = "Autonomous")
// @Disabled
public class LeftEncoder2Cone extends LinearOpMode {

  @Override
  public void runOpMode() {
    // Initialize the hardware variables.
    RobotClass robot = new RobotClass(hardwareMap, true);

    // ! Runs upon initialization
    telemetry.addData("Status", "Initialized");
    telemetry.update();

    robot.BRDrive.setDirection(DcMotor.Direction.FORWARD);
    robot.FLDrive.setDirection(DcMotor.Direction.REVERSE);
    robot.FRDrive.setDirection(DcMotor.Direction.REVERSE);
    robot.BLDrive.setDirection(DcMotor.Direction.FORWARD);

    robot.setToEncoderMode();
    robot.resetDrive();

    while (!isStarted()) {
      telemetry.addData("Position", robot.sleeveDetection.getPosition());
      telemetry.addData("PRELOAD CONE", "PRELOAD IT DUMBASS");
      telemetry.update();
      robot.position = robot.sleeveDetection.getPosition();
    }

    // ! Runs until the end of the match after play is pressed
    waitForStart();
    robot.timeElapsed.reset();

    while (opModeIsActive()) {
      robot.resetDrive();
      robot.ClawServo.setPosition(robot.servoClosePos);
      realSleep(1000, "Close claw", robot);
      robot.SlideMotor.setPower(robot.slidePowerUp);
      realSleep(2000, "Raise slide", robot);
      robot.SlideMotor.setPower(0);
      realSleep(500, "Stop slide", robot);
      encodeGood(0.2, 859, 747, 778, 862);
      encodeGood(0.2, 509, 448, 466, 508);
      robot.stopDrive();
      encodeGood(0.2, -509, -448, -466, -508);
      robot.stopDrive(); //might need to add sleep
      encodeGood(0.2,448,-401,-440,433); //move to pole
      encodeGood(0.2,224,190,197,215); //go forwdard
      robot.stopDrive();
      robot.SlideMotor.setPower(robot.slidePowerDown);
      sleep(1500);
      robot.SlideMotor.setPower(0);

      robot.ClawServo.setPosition(robot.servoOpenPos);
      sleep(300);
      encodeGood(0.2,-100,-100,-100,-100); //,move back was -150 bnefore
      encodeGood(0.2,-448,401,440,-433); //move away from pole


            //NEW CODE STARTS HERE
      encodeGood(0.3,200,200,200,200); //move forward, in line with stack


      //rotate left 90 degrees, facing stack
      encodeGood(0.2, -1000, -1000, 1000, 1000);

      //move forward
      encodeGood(0.4, 1000, 1000, 1000, 1000); //move forward just away from stack
      encodeGood(0.2, 200, 200, 200, 200); //move forward with claw in stack

      //grab stack
      robot.ClawServo.setPosition(robot.servoClosePos);
      sleep(300);

      //raise claw
      robot.SlideMotor.setPower(robot.slidePowerUp); //bring claw in line with high pole
      sleep(1600);

      //move back
      encodeGood(0.2, -200, -200, -200, -200); //move back with claw in stack

      encodeGood(0.3, -1000, -1000, -1000, -1000); //move back far away from stack

      //rotate left
      encodeGood(0.2, 1000, 1000, -1000, -1000); //rotate right 90 degrees, facing high pole

      //move forward
      encodeGood(0.2, 100, 100, 100, 100); //move forward just away from high pole

      //lower claw
      robot.SlideMotor.setPower(robot.slidePowerDown); //lower claw onto high pole
      sleep(1600);

      //release stack
      robot.ClawServo.setPosition(robot.servoOpenPos);
      sleep(300);

      //move back
      encodeGood(0.2, -100, -100, -100, -100); //move back with claw in stack

      //rotate right
      encodeGood(0.2, -1000, -1000, 1000, 1000); //rotate left 90 degrees, facing stack

      //move to stack
      encodeGood(0.4, 1000, 1000, 1000, 1000); //move forward just away from stack
      encodeGood(0.2, 200, 200, 200, 200); //move forward with claw in stack

           //grab stack
      robot.ClawServo.setPosition(robot.servoClosePos);
      sleep(300);

      //raise claw
      robot.SlideMotor.setPower(robot.slidePowerUp); //bring claw in line with high pole
      sleep(1600);

      //move back
      encodeGood(0.2, -200, -200, -200, -200); //move back with claw in stack

      encodeGood(0.3, -1000, -1000, -1000, -1000); //move back far away from stack

      //rotate right
      encodeGood(0.2, 1000, 1000, -1000, -1000); //rotate right 90 degrees, facing high pole

      //move forward
      encodeGood(0.2, 100, 100, 100, 100); //move forward just away from high pole

      //lower claw
      robot.SlideMotor.setPower(robot.slidePowerDown); //lower claw onto high pole
      sleep(1600);

      //release stack
      robot.ClawServo.setPosition(robot.servoOpenPos);
      sleep(300);

      //move back
      encodeGood(0.2, -100, -100, -100, -100); //move back with claw in stack


      //NEW CODE ENDS HERE

      if (robot.position.equals("Left")) { //shouldnt need to be changed much
        // ** Good
        robot.stopDrive();
        encodeGood(0.2, -768, 711, 835, -822); // last one! :D
        robot.stopDrive();
      } else if (robot.position.equals("Center")) {
        // ** Good
        robot.stopDrive();
      } else if (robot.position.equals("Right")) {
        // ** Good
        robot.stopDrive();
        encodeGood(0.2, 780, -741, -808, 861);
        robot.stopDrive();
      }
      robot.stopDrive();
      realSleep(9999999, "Done", robot);
    }
  }

  public void realSleep(int n, String customAdd, RobotClass robot) { // better sleep method, dont use other crappy
    // stuffs
    telemetry.addData("Status", customAdd);
    telemetry.addData("FL Encoder", -robot.FLDrive.getCurrentPosition());
    telemetry.addData("FR Encoder", -robot.FRDrive.getCurrentPosition());
    telemetry.addData("BL Encoder", -robot.BLDrive.getCurrentPosition());
    telemetry.addData("BR Encoder", -robot.BRDrive.getCurrentPosition());
    telemetry.addData("Cone Pos", robot.position);
    telemetry.update();

    sleep(n);
  }

  public void encoderTelemetry(RobotClass robot) {
    telemetry.addData("FL Encoder", robot.FLDrive.getCurrentPosition());
    telemetry.addData("FR Encoder", robot.FRDrive.getCurrentPosition());
    telemetry.addData("BL Encoder", robot.BLDrive.getCurrentPosition());
    telemetry.addData("BR Encoder", robot.BRDrive.getCurrentPosition());
    telemetry.update();
  }
      //tedt so we don need sleep, motors should say "im done" by themselves   
    public void encodeGood( //might need to remove some robot. if thing die
    double power,
    int FLPos,
    int FRPos,
    int BLPos,
    int BRPos
  ) {
    robot.setPos(
      robot.FLDrive.getCurrentPosition() + FLPos,
      robot.FRDrive.getCurrentPosition() + FRPos,
      robot.BLDrive.getCurrentPosition() + BLPos,
      robot.BRDrive.getCurrentPosition() + BRPos
    );
    robot.setDrivePower(power, power, power, power);
    while (opModeIsActive() && robot.BLDrive.isBusy() && robot.FRDrive.isBusy())
    {
      //do nothing or put idle() if bad thing happen
    }
    sleep(15); //just in case motors decide to wibble wobble or something
    robot.setDrivePower(0,0,0,0);
  }
}