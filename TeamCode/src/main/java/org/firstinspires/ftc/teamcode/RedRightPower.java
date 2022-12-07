package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
// import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
// import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

// import org.openftc.easyopencv.OpenCvCamera;
// import org.openftc.easyopencv.OpenCvCameraFactory;
// import org.openftc.easyopencv.OpenCvCameraRotation;

@Autonomous(name = "Red Left Power", group = "Autonomous")
// @Disabled
public class RedRightPower extends LinearOpMode {
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

    // TODO Open CV
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
      // ! New Stuff
      robot.ClawServo.setPosition(robot.servoClosePos);
      realSleep(1000, "Close Servo with cone", robot);
      robot.SlideMotor.setPower(robot.slidePowerUp);
      realSleep(350, "Slide up with cone", robot);
      robot.SlideMotor.setPower(0);
      realSleep(100, "Slide motor stopped", robot);

      robot.setDrivePower(0.25, 0.25, 0.25, 0.25);
      realSleep(2000, "Forward", robot);

      robot.omnidrive(.5, Math.PI / -2, 0);
      realSleep(500, "omni-driving to the left", robot);

      robot.setDrivePower(0.2, 0.2, 0.2, 0.2); // get junction inside of v thing\
      realSleep(250, "onto high junction", robot);

      robot.ClawServo.setPosition(0.7494); // open servo and drop cone on high junction
      realSleep(100, "Servo open", robot);

      robot.setDrivePower(-0.2, -0.2, -0.2, -0.2); // back up
      realSleep(500, "backing up", robot);
      robot.SlideMotor.setPower(-1);
      realSleep(2400, "Slide motor down", robot); // sligjhtly less so dont de-tension cable
      robot.SlideMotor.setPower(0);
      realSleep(100, "Slide motor stopped", robot);
      robot.ClawServo.setPosition(0.355);
      realSleep(150, "Servo closed", robot);

      // here, robot should be just in front of the high junction and ready 2 park
      // ALREADY BACKED UP!
      if (robot.position.equals("Left")) {
        // drive to left (position one)
        robot.omnidrive(.2, (Math.PI / 2) * -1, 0);
        realSleep(350, "omni to location one", robot);
        robot.setDrivePower(0, 0, 0, 0);
        realSleep(100, "parked in location one", robot);

      } else if (robot.position.equals("Right")) {
        // drive to right (position three)
        robot.omnidrive(.2, (Math.PI / 2), 0);
        realSleep(700, "omni to location three", robot);
        robot.setDrivePower(0, 0, 0, 0);
        realSleep(100, "parked in location three", robot);

      } else {
        // drive to center (position two)
        robot.omnidrive(.2, (Math.PI / 2), 0);
        realSleep(350, "omni to location two", robot);
        robot.setDrivePower(0, 0, 0, 0);
        realSleep(100, "parked in location two", robot);
      }

      realSleep(9999999, "Done", robot);
    }
  }

  public void realSleep(int n, String customAdd, RobotClass robot) { // better sleep method, dont use other crappy
                                                                     // stuffs
    telemetry.addData("Status", customAdd);
    telemetry.addData("Claw Servo Position", robot.ClawServo.getPosition());
    telemetry.addData("Slide Touch Sensor", !(robot.SlideTouchSensor.getState()));
    telemetry.addData("Elapsed Time", robot.timeElapsed.toString());

    sleep(n);

    // telemetry.addData("Front left/Right", "%4.2f, %4.2f", robot.FLPower,
    // robot.FRPower);
    // telemetry.addData("Back left/Right", "%4.2f, %4.2f", robot.BLPower,
    // robot.BRPower);
    telemetry.update();
  }

}
