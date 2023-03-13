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

@Autonomous(name = "Left Far High", group = "Autonomous")
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
      for (int i = 0; i < 10; i++) {
        telemetry.addData("PRELOAD CONE", "PRELOAD IT DUMBASS");
      }
      telemetry.update();
      robot.position = robot.sleeveDetection.getPosition();
      robot.ClawServo.setPosition(robot.servoClosePos);
    }

    // ! Runs until the end of the match after play is pressed
    waitForStart();
    robot.timeElapsed.reset();

    while (opModeIsActive()) {
      robot.resetDrive();
      robot.ClawServo.setPosition(robot.servoClosePos);
      sleep(100);
      robot.encoderDrive(
          0.26,
          1780,
          1770,
          1770,
          1780,
          robot.slidePowerUp,
          2200.0); // Forward to cone (used to be 2000)
      robot.encoderDrive(0.2, -180, -170, -170, -180); // Back away from cone
      robot.encoderDrive(0.2, 400, -400, -400, 400); // Omni to pole (was 420)
      robot.encoderDrive(0.2, 120, 120, 120, 120); // Forward to pole
      robot.SlideMotor.setPower(robot.slidePowerDown - 0.1); // Lower onto pole
      sleep(900);
      robot.SlideMotor.setPower(0); // Stop lowering
      robot.ClawServo.setPosition(robot.servoOpenPos); // Open servo
      robot.encoderDrive(0.2, -120, -120, -120, -120); // Back from pole




      // //+1 Starts Here
      // robot.SlideMotor.setPower(robot.slidePowerDown - 0.1); // Lower onto pole
      // sleep(1000);
      // robot.SlideMotor.setPower(0); // Stop lowering
      // robot.encoderDrive(0.2,-645,640,-670,640); //was 650 all-around
      // //start daniel code
      // robot.encoderDrive(0.2,1166,1068,1128,1139, robot.slidePowerDown,600); //was 650 all-around
      // sleep(99999);
      // robot.SlideMotor.setPower(robot.slidePowerDown - 0.1); // Lower onto pole
      // sleep(600);
      // robot.SlideMotor.setPower(0); // Stop lowering
      // robot.ClawServo.setPosition(robot.servoClosePos);
      // sleep(300);
      // robot.SlideMotor.setPower(robot.slidePowerUp);
      // sleep(300);
      // robot.SlideMotor.setPower(0);
      // robot.encoderDrive(0.3,-1166,-1068,-1128,-1139, robot.slidePowerUp,600);
      // robot.encoderDrive(0.2,645,-640,670,-640);
      // robot.encoderDrive(0.2,100,100,100,100); 
      // robot.ClawServo.setPosition(robot.servoOpenPos);
      // sleep(100);



 
      if (robot.position.equals("Left")) {
        // ** Good
        robot.stopDrive();
        robot.encoderDrive(0.2, -1250, 1150, 1150, -1250); // last one! :D
        // realSleep(2900, "omni to pole", robot);
        robot.stopDrive();
      } else if (robot.position.equals("Center")) {
        // ** Good
        robot.encoderDrive(0.2, -450, 450, 450, -450); // last one! :D
        robot.stopDrive();
      } else if (robot.position.equals("Right")) {
        // ** Good
        robot.stopDrive();
        robot.encoderDrive(0.2, 340, -340, -340, 340);
        // realSleep(2900, "omni to pole", robot);
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
}
