package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

@TeleOp(name = "Auto Encoder Dev", group = "Autonomous Dev")
public class AutoEncoderDev extends LinearOpMode {
  @Override
  public void runOpMode() {
    // Initialize the hardware variables.
    RobotClass robot = new RobotClass(hardwareMap, false);

    // ! Runs upon initialization
    telemetry.addData("Status", "Initialized");
    telemetry.update();

    // robot.BRDrive.setDirection(DcMotor.Direction.FORWARD);
    // robot.robot.FLDrive.setDirection(DcMotor.Direction.REVERSE);
    // robot.FRDrive.setDirection(DcMotor.Direction.REVERSE);
    // robot.BLDrive.setDirection(DcMotor.Direction.FORWARD);

    robot.setToEncoderMode();
    robot.resetDrive();

    // Initialize drive variables
    float vertical;
    float horizontal;
    float pivot;
    double speedScalar;

    // ! Runs until the end of the match after play is pressed
    waitForStart();
    robot.timeElapsed.reset();

    while (opModeIsActive()) {
      double max;

      vertical = gamepad1.left_stick_y;
      horizontal = -gamepad1.left_stick_x;
      pivot = -gamepad1.right_stick_x;

      speedScalar = 0.2; // used to be .65 for fast

      double FRPower = ((-pivot + (vertical - horizontal)) * speedScalar);
      double BRPower = ((-pivot + vertical + horizontal) * speedScalar);
      double FLPower = ((pivot + vertical + horizontal) * speedScalar);
      double BLPower = ((pivot + (vertical - horizontal)) * speedScalar);

      // ? Nerd stuff to make sure the robot doesn't go too fast
      max = Math.max(Math.abs(FLPower), Math.abs(FRPower));
      max = Math.max(max, Math.abs(BLPower));
      max = Math.max(max, Math.abs(BRPower));

      if (max > 1.0) {
        FLPower /= max;
        FRPower /= max;
        BLPower /= max;
        BRPower /= max;
      }
      // ? Nerd stuff ends here

      robot.setDrivePower(FLPower, FRPower, BLPower, BRPower);

      if (gamepad1.right_stick_button){
          robot.resetDrive();
          robot.setToPowerMode();
      }

      if (gamepad1.left_stick_button){
          robot.resetDrive();
          robot.setToPowerMode();
      }

      telemetry.addData("FL Encoder", -robot.FLDrive.getCurrentPosition());
      telemetry.addData("FR Encoder", -robot.FRDrive.getCurrentPosition());
      telemetry.addData("BL Encoder", -robot.BLDrive.getCurrentPosition());
      telemetry.addData("BR Encoder", -robot.BRDrive.getCurrentPosition());
      telemetry.update();
    }
  }
}
