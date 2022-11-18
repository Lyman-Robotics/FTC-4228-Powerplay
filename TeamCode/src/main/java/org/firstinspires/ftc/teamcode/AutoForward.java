package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

@Autonomous(name = "Forward & Back", group = "Autonomous")
public class AutoForward extends LinearOpMode {
  @Override
  public void runOpMode() {
    // Initialize the hardware variables.
    RobotClass robot = new RobotClass(hardwareMap);

    // ! Runs upon initialization
    telemetry.addData("Status", "Initialized");
    telemetry.update();
    waitForStart();
    robot.timeElapsed.reset();

    robot.ClawServo.setPosition(0.7494);

    robot.BRDrive.setDirection(DcMotor.Direction.FORWARD);
    robot.FLDrive.setDirection(DcMotor.Direction.REVERSE);
    robot.FRDrive.setDirection(DcMotor.Direction.REVERSE);
    robot.BLDrive.setDirection(DcMotor.Direction.FORWARD);

    // Initialize drive variables

    // ! Runs until the end of the match after play is pressed
    while (opModeIsActive()) {
      robot.setDrivePower(0.25, 0.25, 0.25, 0.25);
      sleep(2000);
      robot.setDrivePower(0, 0, 0, 0);
      sleep(500);
      robot.setDrivePower(-0.25, -0.25, -0.25, -0.25);
      sleep(2200);
      robot.setDrivePower(0, 0, 0, 0);

      sleep(9999999);
    }
  }
}
