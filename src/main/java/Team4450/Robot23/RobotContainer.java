
package Team4450.Robot23;

import static Team4450.Robot23.Constants.*;

import java.io.IOException;
import java.nio.file.Path;

import com.pathplanner.lib.PathConstraints;
import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;

import Team4450.Lib.CameraFeed;
import Team4450.Lib.XboxController;
import Team4450.Lib.MonitorPDP;
import Team4450.Lib.NavX;
import Team4450.Lib.Util;
import Team4450.Robot23.commands.AlignToTape;
import Team4450.Robot23.commands.AlignToTapeLL;
import Team4450.Robot23.commands.CloseClaw;
import Team4450.Robot23.commands.DriveArm;
import Team4450.Robot23.commands.DriveClaw;
import Team4450.Robot23.commands.DriveCommand;
import Team4450.Robot23.commands.DriveWinch;
import Team4450.Robot23.commands.DropArm;
import Team4450.Robot23.commands.ExtendArm;
import Team4450.Robot23.commands.FeedStation;
import Team4450.Robot23.commands.HoldWinchPosition;
import Team4450.Robot23.commands.IntakeCone;
import Team4450.Robot23.commands.IntakeCube;
import Team4450.Robot23.commands.LowerArm;
import Team4450.Robot23.commands.OpenClaw;
import Team4450.Robot23.commands.ParkWheels;
import Team4450.Robot23.commands.RaiseArm;
import Team4450.Robot23.commands.RaiseArmStart;
import Team4450.Robot23.commands.RetractArm;
import Team4450.Robot23.commands.ScoreHigh;
import Team4450.Robot23.commands.ScoreMid;
import Team4450.Robot23.commands.SetToStartPositionCommand;
import Team4450.Robot23.commands.Utility.NotifierCommand;
import Team4450.Robot23.commands.autonomous.DriveOut;
import Team4450.Robot23.commands.autonomous.AutoScoreHigh;
import Team4450.Robot23.commands.autonomous.AutoScoreHighNoDrive;
import Team4450.Robot23.commands.autonomous.AutoScoreLow;
import Team4450.Robot23.commands.autonomous.TestAuto1;
import Team4450.Robot23.commands.autonomous.TestAuto3;
import Team4450.Robot23.commands.autonomous.TestAuto4;
import Team4450.Robot23.subsystems.Arm;
import Team4450.Robot23.subsystems.Claw;
import Team4450.Robot23.subsystems.DriveBase;
import Team4450.Robot23.subsystems.Intake;
import Team4450.Robot23.subsystems.LimeLight;
import Team4450.Robot23.subsystems.PhotonVision;
import Team4450.Robot23.subsystems.ShuffleBoard;
import Team4450.Robot23.subsystems.Winch;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.PowerDistribution;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.util.sendable.SendableRegistry;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryUtil;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.Trigger;

/**
 * This class is where the bulk of the robot should be declared.  Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls).  Instead, the structure of the robot
 * (including subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer 
{
	// Subsystems.

	public static ShuffleBoard	shuffleBoard;
	public static DriveBase 	driveBase;
	public static Winch			winch;
	public static Arm			arm;
	//public static Claw			claw;
	public static Intake		intake;
	public static PhotonVision	photonVision;
	public static LimeLight		limeLight;

	// Subsystem Default Commands.

    // Persistent Commands.

	// Some notes about Commands.
	// When a Command is created with the New operator, its constructor is called. When the
	// command is added to the Scheduler to be run, its initialize method is called. Then on
	// each scheduler run, as long as the command is still scheduled, its execute method is
	// called followed by isFinished. If isFinished it false, the command remains in the
	// scheduler list and on next run, execute is called followed by isFinihsed. If isFinished
	// returns true, the end method is called and the command is removed from the scheduler list.
	// Now if you create another instance with new, you get the constructor again. But if you 
	// are re-scheduling an existing command instance (like the ones above), you do not get the
	// constructor called, but you do get initialize called again and then on to execute & etc.
	// So this means you have to be careful about command initialization activities as a persistent
	// command in effect has two lifetimes (or scopes): Class global and each new time the command
	// is scheduled. Note the FIRST doc on the scheduler process is not accurate as of 2020.
	
	// GamePads. 2 Game Pads use RobotLib XboxController wrapper class for some extra features.
	// Note that button responsiveness may be slowed as the schedulers command list gets longer 
	// or commands get longer as buttons are processed once per scheduler run.
	
	private XboxController			driverPad =  new XboxController(DRIVER_PAD);
	public static XboxController	utilityPad = new XboxController(UTILITY_PAD);

	//private AnalogInput			pressureSensor = new AnalogInput(PRESSURE_SENSOR);
	  
	//private PowerDistribution	pdp = new PowerDistribution(0, PowerDistribution.ModuleType.kCTRE);
	private PowerDistribution	pdp = new PowerDistribution(REV_PDB, PowerDistribution.ModuleType.kRev);

	// PneumaticsControlModule class controls the PCM. New for 2022.
	//private PneumaticsControlModule	pcm = new PneumaticsControlModule(COMPRESSOR);

	// Navigation board.
	public static NavX			navx;

	private MonitorPDP     		monitorPDPThread;
	//private MonitorCompressor	monitorCompressorThread;
    private CameraFeed			cameraFeed;
    
	// Trajectories.
    public static Trajectory    		testTrajectory;
	public static PathPlannerTrajectory	ppTestTrajectory;

    // List of autonomous programs. Any change here must be reflected in getAutonomousCommand()
    // and setAutoChoices() which appear later in this class.
	private enum AutoProgram
	{
		NoProgram,
		DriveOut,
		ScoreLow,
		ScoreHigh,
		ScoreHighNoDrive,
		TestAuto1,
		TestAuto3,
		TestAuto4
	}

	// Classes to access drop down lists on Driver Station.
	private static SendableChooser<AutoProgram>	autoChooser;
	private static SendableChooser<Integer>		startingPoseChooser;

	/**
	 * The container for the robot. Contains subsystems, Opertor Interface devices, and commands.
	 */
	public RobotContainer() throws Exception
	{
		Util.consoleLog();

		// Get information about the match environment from the Field Control System.
      
		getMatchInformation();

		// Read properties file from RoboRio "disk". If we fail to open the file,
		// log the exception but continue and default to competition robot.
      
		try {
			robotProperties = Util.readProperties();
		} catch (Exception e) { Util.logException(e);}

		// Is this the competition or clone robot?
   		
		if (robotProperties == null || robotProperties.getProperty("RobotId").equals("comp"))
			isComp = true;
		else
			isClone = true;
 		
		// Set compressor enabled switch on dashboard from properties file.
		// Later code will read that setting from the dashboard and turn 
		// compressor on or off in response to dashboard setting.
 		
		boolean compressorEnabled = true;	// Default if no property.

		if (robotProperties != null) 
			compressorEnabled = Boolean.parseBoolean(robotProperties.getProperty("CompressorEnabledByDefault"));
		
		SmartDashboard.putBoolean("CompressorEnabled", compressorEnabled);

		// Reset PDB & PCM sticky faults.
    
		resetFaults();

		// Create NavX object here since must done before CameraFeed is created (don't remember why).
        // Navx calibrates at power on and must complete before robot moves. Takes ~1 second for 2nd
        // generation Navx ~15 seconds for classic Navx. We assume there will be enough time between
        // power on and our first movement because normally things don't happen that fast

		// Warning: The navx instance is shared with the swerve drive code. Resetting or otherwise
		// manipulating the navx (as opposed to just reading data) may crash the swerve drive code.

		navx = NavX.getInstance(NavX.PortType.SPI);

		// Add navx as a Sendable. Updates the dashboard heading indicator automatically.
 		
		SmartDashboard.putData("Gyro2", navx);

		// Invert driving joy sticks Y axis so + values mean forward.
		// Invert driving joy sticks X axis so + values mean right.
	  
		driverPad.invertY(true);
		driverPad.invertX(true);		

		// Create subsystems prior to button mapping.

		shuffleBoard = new ShuffleBoard();
		driveBase = new DriveBase();
		winch = new Winch();
		arm = new Arm(winch);
		//claw = new Claw();
		intake = new Intake();
		photonVision = new PhotonVision();
		limeLight = new LimeLight();

		// Create any persistent commands.

		// Set any subsystem Default commands.

		// Set the default drive command. This command will be scheduled automatically to run
		// every teleop period and so use the gamepad joy sticks to drive the robot. We pass the GetY()
		// functions on the Joysticks as a DoubleSuppier. The point of this is removing the direct 
		// connection between the Drive and XboxController classes. We are in effect passing functions 
		// into the Drive command so it can read the values later when the Drive command is executing 
		// under the Scheduler. Drive command code does not have to know anything about the JoySticks 
		// (or any other source) but can still read them. We can pass the DoubleSupplier two ways. First
		// is with () -> lambda expression which wraps the getLeftY() function in a DoubleSupplier instance.
		// Second is using the convenience method getRightYDS() which returns getRightY() as a DoubleSupplier. 
		// We show both ways here as an example.

		// The joystick controls for driving:
		// Left stick Y axis -> forward and backwards movement (throttle)
		// Left stick X axis -> left and right movement (strafe)
		// Right stick X axis -> rotation
		// Note: X and Y axis on stick is opposite X and Y axis on the WheelSpeeds object
		// and the odometry pose2d classes.
		// Wheelspeeds +X axis is down the field away from alliance wall. +Y axis is left
		// when standing at alliance wall looking down the field.
		// This is handled here by swapping the inputs. Note that first axis parameter below
		// is the X wheelspeeds input and the second is Y wheelspeeds input.

		// Note that field oriented driving does the movements in relation to the field. So
		// throttle is always down the field and back and strafe is always left right from
		// the down the field axis, no matter which way the robot is pointing. Robot oriented
		// driving movemments are in relation to the direction the robot is currently pointing.

		driveBase.setDefaultCommand(new DriveCommand(
				driveBase,
				() -> driverPad.getLeftY(),	// Throttle
				() -> driverPad.getLeftX(),	// Strafe
				driverPad.getRightXDS(),	// Rotation
				driverPad));

		winch.setDefaultCommand(new DriveWinch(winch, () -> utilityPad.getRightY()));

		//claw.setDefaultCommand(new DriveClaw(claw, () -> utilityPad.getRightX()));

		arm.setDefaultCommand(new DriveArm(arm, () -> utilityPad.getLeftY(), utilityPad));

		// Start the compressor, PDP and camera feed monitoring Tasks.

   		// monitorCompressorThread = MonitorCompressor.getInstance(pressureSensor);
   		// monitorCompressorThread.setDelay(1.0);
   		// monitorCompressorThread.SetLowPressureAlarm(50);
   		// monitorCompressorThread.start();
		
   		monitorPDPThread = MonitorPDP.getInstance(pdp);
   		monitorPDPThread.start();
		
		// Start camera server thread using our class for usb cameras.
    
		cameraFeed = CameraFeed.getInstance(); 
		cameraFeed.start();
 		
		// Log info about NavX.
	  
		navx.dumpValuesToNetworkTables();
 		
		if (navx.isConnected())
			Util.consoleLog("NavX version=%s", navx.getAHRS().getFirmwareVersion());
		else
		{
			Exception e = new Exception("NavX is NOT connected!");
			Util.logException(e);
		}
        
        // Configure autonomous routines and send to dashboard.

		setAutoChoices();

		setStartingPoses();

		// Configure the button bindings.
		
        configureButtonBindings();
        
        // Load any trajectory files in a separate thread on first scheduler run.
        // We do this because trajectory loads can take up to 10 seconds to load so we want this
        // being done while we are getting started up. Hopefully will complete before we are ready to
        // use the trajectory.
		
		// NotifierCommand loadTrajectory = new NotifierCommand(this::loadTestTrajectory, 0);
        // loadTrajectory.setRunWhenDisabled(true);
        // CommandScheduler.getInstance().schedule(loadTrajectory);
		
		// //testTrajectory = loadTrajectoryFile("Slalom-1.wpilib.json");
		
		// loadTrajectory = new NotifierCommand(this::loadPPTestTrajectory, 0);
        // loadTrajectory.setRunWhenDisabled(true);
        // CommandScheduler.getInstance().schedule(loadTrajectory);

		//PathPlannerTrajectory ppTestTrajectory = loadPPTrajectoryFile("richard");
	}

	/**
	 * Use this method to define your button->command mappings.
     * 
     * These buttons are for real robot driver station with 3 sticks and launchpad.
	 * The launchpad makes the colored buttons look like a joystick.
	 */
	private void configureButtonBindings() 
	{
		Util.consoleLog();
	  
		// ------- Driver pad buttons -------------
		
		// For simple functions, instead of creating commands, we can call convenience functions on
		// the target subsystem from an InstantCommand. It can be tricky deciding what functions
		// should be an aspect of the subsystem and what functions should be in Commands...

		// Advance DS tab display.
		new Trigger(() -> driverPad.getPOVAngle(90))
			.onTrue(new InstantCommand(shuffleBoard::switchTab));
        
		// Set wheels to starting position.
		new Trigger(() -> driverPad.getStartButton())
			.onTrue(new SetToStartPositionCommand(driveBase));

	    // Back button toggles field/robot oriented driving mode.
    	new Trigger(() -> driverPad.getBackButton())
        	.onTrue(new InstantCommand(driveBase::toggleFieldOriented));
		
		// NOTE: Left bumper engages "slow" mode and is defined in DriveCommand.

		// Change camera feed. 
		new Trigger(() -> driverPad.getRightBumper())
    		.onTrue(new InstantCommand(cameraFeed::ChangeCamera));

		// Reset yaw angle to zero.
		new Trigger(() -> driverPad.getPOVAngle(180))
    		.onTrue(new InstantCommand(driveBase::resetYaw));

		// Toggle drive motors between brake and coast.
		new Trigger(() -> driverPad.getBButton())
    		.onTrue(new InstantCommand(driveBase::toggleBrakeMode));

		// Toggle Limelight LED.
		new Trigger(() -> driverPad.getAButton())
    		.onTrue(new InstantCommand(photonVision::toggleLedMode));

		// Set drive wheels to parking orientation.
		new Trigger(() -> driverPad.getXButton())
    		.onTrue(new ParkWheels(driveBase));

		// Reset drive wheel distance traveled.
		new Trigger(() -> driverPad.getPOVAngle(270))
    		.onTrue(new InstantCommand(driveBase::resetDistanceTraveled));

		// Apply holding voltage to winch.
		new Trigger(() -> driverPad.getRightTrigger())		
			.onTrue(new InstantCommand(winch::toggleHoldPosition));

		// Toggle automatic tape target alignment command.
		new Trigger(() -> driverPad.getLeftTrigger())		
			.toggleOnTrue(new AlignToTape(photonVision, driveBase));

		// new Trigger(() -> driverPad.getLeftTrigger())		
		// 	.toggleOnTrue(new AlignToTapeLL(limeLight, driveBase));		
		
		// -------- Utility pad buttons ----------
		// What follows is an example from 2022 robot:
		// Toggle extend Pickup.
		// So we show 3 ways to control the pickup. A regular command that toggles pickup state,
		// an instant command that calls a method on Pickup class that toggles state and finally
		// our special notifier variant that runs the Pickup class toggle method in a separate
		// thread. So we show all 3 methods as illustration but the reason we tried 3 methods is
		// that the pickup retraction action takes almost 1 second (due apparently to some big
		// overhead in disabling the electric eye interrupt) and triggers the global and drivebase
		// watchdogs (20ms). Threading does not as the toggle method is not run on the scheduler thread.
		// Also, any action that operates air valves, there is a 50ms delay in the ValveDA and SA
		// classes to apply power long enough so that the valve slides move far enough to open/close.
		// So any function that operates valves will trigger the watchdogs. Again, the watchdog 
		// notifications are only a warning (though too much delay on main thread can effect robot
		// operation) they can fill the Riolog to the point it is not useful.
		// Note: the threaded command can only execute a runable (function on a class) not a Command.
		
		// Toggle pickup deployment
		//new Trigger(() -> utilityPad.getLeftBumper())
        	//.onTrue(new PickupDeploy(pickup));		
			//.onTrue(new InstantCommand(pickup::toggleDeploy, pickup));
		//	.onTrue(new NotifierCommand(pickup::toggleDeploy, 0.0, "DeployPickup", pickup));

		// Start or stop (if already in progress), the command to raise arm to start position.
		new Trigger(() -> utilityPad.getPOVAngle(0)).toggleOnTrue(new RaiseArmStart(winch));

		// Start or stop (if already in progress), the command to drop arm to low position.
		//new Trigger(() -> utilityPad.getRightBumper()).toggleOnTrue(new DropArm(winch, arm));

		// Start or stop (if already in progress), the command to retract arm to inward position.
		new Trigger(() -> utilityPad.getPOVAngle(180)).toggleOnTrue(new RetractArm(arm));

		// Start or stop (if already in progress), the command to drop loaded game piece.
		new Trigger(() -> utilityPad.getRightTrigger()).toggleOnTrue(new InstantCommand(intake::dropGamePiece));

		// Start or stop (if already in progress), the command to intake cube.
		new Trigger(() -> utilityPad.getLeftBumper()).toggleOnTrue(new IntakeCube(intake));

		// Start or stop (if already in progress), the command to intake cone.
		new Trigger(() -> utilityPad.getLeftTrigger()).toggleOnTrue(new IntakeCone(intake));

		// Start or stop (if already in progress), the command to lower the arm to scoring position 1.
		//new Trigger(() -> utilityPad.getYButton()).toggleOnTrue(lowerArm1);

		// Start or stop (if already in progress), the command to position for feeder pickup.
		new Trigger(() -> utilityPad.getXButton()).toggleOnTrue(new FeedStation(winch, arm, intake));

		// Start or stop (if already in progress), the command to extend the arm to mid scoring position.
		new Trigger(() -> utilityPad.getBButton()).toggleOnTrue(new ScoreMid(winch, arm, intake));

		// Start or stop (if already in progress), the command to extend the arm to high scoring position.
		//new Trigger(() -> utilityPad.getYButton()).toggleOnTrue(new ScoreHigh(winch, arm, intake));

		// Apply holding voltage to winch.
		new Trigger(() -> utilityPad.getYButton()).onTrue(new InstantCommand(winch::toggleHoldPosition));

		//new Trigger(() -> utilityPad.getPOVAngle(270)).onTrue(new InstantCommand(arm::resetPosition));
		new Trigger(() -> utilityPad.getBackButton()).onTrue(new InstantCommand(arm::resetPosition));
	}

	/**
	 * Use this to pass the autonomous command(s) to the main {@link Robot} class.
	 * Determines which auto command from the selection made by the operator on the
	 * DS drop down list of commands.
	 * @return The command to run in autonomous
	 */
	public Command getAutonomousCommand() 
	{
		AutoProgram		program = AutoProgram.NoProgram;
		Pose2d			startingPose = DEFAULT_STARTING_POSE;
		Integer			startingPoseIndex = 0;
		Command			autoCommand = null;
		
		Util.consoleLog();

		try
		{
			program = autoChooser.getSelected();

			startingPoseIndex = startingPoseChooser.getSelected();

			startingPose = STARTING_POSES[startingPoseIndex];

			// Adjust Y position for Red side of the field. Hopefully this works.
			
			if (alliance == Alliance.Red) startingPose = new Pose2d(startingPose.getX(), 8.014 - startingPose.getY(), 
																	startingPose.getRotation());
		}
		catch (Exception e)	{ Util.logException(e); }
		
		switch (program)
		{
			case NoProgram:
				autoCommand = null;
				break;
 				
			case DriveOut:
				autoCommand = new DriveOut(driveBase, startingPose, startingPoseIndex);
				break;
 				
			case ScoreLow:
				autoCommand = new AutoScoreLow(driveBase, winch, arm, intake, startingPose, startingPoseIndex);
				break;
 				
			case ScoreHigh:
				autoCommand = new AutoScoreHigh(driveBase, winch, arm, intake, startingPose, startingPoseIndex);
				break;
 				
			case ScoreHighNoDrive:
				autoCommand = new AutoScoreHighNoDrive(driveBase, winch, arm, intake, startingPose, startingPoseIndex);
				break;
				
			case TestAuto1:
			 	autoCommand = new TestAuto1(driveBase, startingPose);
			 	break;
 				
			case TestAuto3:
			 	autoCommand = new TestAuto3(driveBase, startingPose);
			 	break;
 				
			case TestAuto4:
			 	autoCommand = new TestAuto4(driveBase, startingPose);
			 	break;
		}
        
		return autoCommand;
	}
  
    // Configure SendableChooser (drop down list on dashboard) with auto program choices and
	// send them to SmartDashboard/ShuffleBoard.
	
	private static void setAutoChoices()
	{
		Util.consoleLog();
		
		autoChooser = new SendableChooser<AutoProgram>();
		
		SendableRegistry.add(autoChooser, "Auto Program");
		autoChooser.setDefaultOption("No Program", AutoProgram.NoProgram);
		autoChooser.addOption("Drive Out", AutoProgram.DriveOut);		
		autoChooser.addOption("Score Low", AutoProgram.ScoreLow);		
		autoChooser.addOption("Score High", AutoProgram.ScoreHigh);		
		autoChooser.addOption("Score High No Drive", AutoProgram.ScoreHighNoDrive);		
		autoChooser.addOption("Test Auto 1", AutoProgram.TestAuto1);		
		autoChooser.addOption("Test Auto 3", AutoProgram.TestAuto3);		
		autoChooser.addOption("Test Auto 4", AutoProgram.TestAuto4);		
				
		SmartDashboard.putData(autoChooser);
	}
  
    // Configure SendableChooser (drop down list on dashboard) with starting pose choices and
	// send them to SmartDashboard/ShuffleBoard.
	
	private void setStartingPoses()
	{
		Util.consoleLog();

		startingPoseChooser = new SendableChooser<Integer>();
		
		SendableRegistry.add(startingPoseChooser, "Start Position");
		startingPoseChooser.setDefaultOption("None", 0);

		for (Integer i = 1; i < 10; i++) startingPoseChooser.addOption(i.toString(), i);		
		
		SmartDashboard.putData(startingPoseChooser);
	}

	/**
	 *  Get and log information about the current match from the FMS or DS.
	 */
	public void getMatchInformation()
	{
		alliance = DriverStation.getAlliance().orElse(Alliance.Blue);
  	  	location = DriverStation.getLocation().orElse(0);
  	  	eventName = DriverStation.getEventName();
	  	matchNumber = DriverStation.getMatchNumber();
	  	gameMessage = DriverStation.getGameSpecificMessage();
    
	  	Util.consoleLog("Alliance=%s, Location=%d, FMS=%b event=%s match=%d msg=%s", 
    		  		   alliance.name(), location, DriverStation.isFMSAttached(), eventName, matchNumber, 
    		  		   gameMessage);
	}
		
	/**
	 * Reset sticky faults in PDP and PCM and turn compressor on/off as
	 * set by switch on DS.
	 */
	public void resetFaults()
	{
		// This code turns on/off the automatic compressor management if requested by DS. Putting this
		// here is a convenience since this function is called at each mode change.
		// if (SmartDashboard.getBoolean("CompressorEnabled", true)) 
		// 	pcm.enableCompressorDigital();
		// else
		// 	pcm.disableCompressor();
		
		pdp.clearStickyFaults();
		//pcm.clearAllStickyFaults();
		
		if (monitorPDPThread != null) monitorPDPThread.reset();
    }
         
    /**
     * Loads a Pathweaver path file into a trajectory.
     * @param fileName Name of file. Will automatically look in deploy directory.
     * @return The path's trajectory.
     */
    public static Trajectory loadTrajectoryFile(String fileName)
    {
        Trajectory  trajectory;
        Path        trajectoryFilePath;

        try 
        {
          trajectoryFilePath = Filesystem.getDeployDirectory().toPath().resolve("paths/" + fileName);

          Util.consoleLog("loading trajectory: %s", trajectoryFilePath);
          
          trajectory = TrajectoryUtil.fromPathweaverJson(trajectoryFilePath);
        } catch (IOException ex) {
		  Util.consoleLog("Unable to open trajectory: " + ex.toString());
          throw new RuntimeException("Unable to open trajectory: " + ex.toString());
        }

        Util.consoleLog("trajectory loaded: %s", fileName);

        return trajectory;
    }
         
	private void loadTestTrajectory()
	{
		testTrajectory = loadTrajectoryFile("Slalom-1.wpilib.json");
	}

	/**
     * Loads a PathPlanner path file into a path planner trajectory.
     * @param fileName Name of file. Will automatically look in deploy directory and add the .path ext.
     * @return The path's trajectory.
     */
    public static PathPlannerTrajectory loadPPTrajectoryFile(String fileName)
    {
        PathPlannerTrajectory  	trajectory;
        Path        			trajectoryFilePath;

		// We fab up the full path for tracing but the loadPath() function does it's own
		// thing constructing a path from just the filename.
		trajectoryFilePath = Filesystem.getDeployDirectory().toPath().resolve("pathplanner/" + fileName + ".path");

		Util.consoleLog("loading PP trajectory: %s", trajectoryFilePath);
		
		trajectory = PathPlanner.loadPath(fileName,
										  new PathConstraints(MAX_WHEEL_SPEED, MAX_WHEEL_ACCEL));

		if (trajectory == null) 
		{
			Util.consoleLog("Unable to open pp trajectory: " + fileName);
			throw new RuntimeException("Unable to open PP trajectory: " + fileName);
		}

        Util.consoleLog("PP trajectory loaded: %s", fileName);

        return trajectory;
    }

	private void loadPPTestTrajectory()
	{
		ppTestTrajectory = loadPPTrajectoryFile("Test-Path");
	}
}
