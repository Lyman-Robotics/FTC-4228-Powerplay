package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

@Autonomous(name = "Autonomous Dev", group = "Autonomous Dev")
public class AutonomousDev extends LinearOpMode {
  @Override
  public void runOpMode() {
    // Initialize the hardware variables.
    RobotClass robot = new RobotClass(hardwareMap);

    // ! Runs upon initialization
    telemetry.addData("Status", "Initialized");
    telemetry.update();
    waitForStart();
    robot.timeElapsed.reset();

    robot.BRDrive.setDirection(DcMotor.Direction.FORWARD);
    robot.FLDrive.setDirection(DcMotor.Direction.REVERSE);
    robot.FRDrive.setDirection(DcMotor.Direction.REVERSE);
    robot.BLDrive.setDirection(DcMotor.Direction.FORWARD);

    robot.ClawServo.setPosition(0.7494); // Init position of servo

    // Initialize drive variables

    // ! Runs until the end of the match after play is pressed
    while (opModeIsActive()) {
      if (gamepad1.a) {
        ElapsedTime timeElapsedNew = new ElapsedTime();
        telemetry.addData("Time", timeElapsedNew);
      }

      if (gamepad1.left_stick_y > 0) {
        robot.setDrivePower(0.25, 0.25, 0.25, 0.25);
      } else if (gamepad1.left_stick_y < 0) {
        robot.setDrivePower(-0.25, -0.25, -0.25, -0.25);
      } else if (gamepad1.left_stick_x > 0) {
        robot.omnidrive(0.25, (Math.PI / 2), 0);
      } else if (gamepad1.left_stick_x < 0) {
        robot.omnidrive(-0.25, (Math.PI / 2), 0);
      } else {
        robot.setDrivePower(0, 0, 0, 0);
      }
    }

    realSleep(9999999, "Done", robot);
  }

  public void realSleep(int n, String customAdd, RobotClass robot) {
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
