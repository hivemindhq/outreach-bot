package org.firstinspires.ftc.teamcode

import com.acmerobotics.dashboard.FtcDashboard
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry
import com.arcrobotics.ftclib.geometry.Pose2d
import com.qualcomm.hardware.bosch.BNO055IMU
import com.qualcomm.hardware.lynx.LynxModule
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode
import com.qualcomm.robotcore.hardware.HardwareMap
import com.qualcomm.robotcore.hardware.configuration.LynxConstants
import com.qualcomm.robotcore.util.ElapsedTime
import org.firstinspires.ftc.robotcore.external.Telemetry
import org.firstinspires.ftc.teamcode.lib.math.Pose
import org.firstinspires.ftc.teamcode.lib.types.Alliance
import org.firstinspires.ftc.teamcode.lib.types.Env
import org.firstinspires.ftc.teamcode.lib.types.OpModeType
import org.firstinspires.ftc.teamcode.subsystem.DepositSubsystem
import org.firstinspires.ftc.teamcode.subsystem.IntakeSubsystem
import org.firstinspires.ftc.teamcode.subsystem.drive.SwerveDriveTrain

/**
 * FTC 23396 Outreach Bot (prev. Centerstage Bot)
 */
class OutreachBot(
    val startPose: Pose2d,
    val opModeType: OpModeType,
    val hardwareMap: HardwareMap,
    val alliance: Alliance,
    var telemetry: Telemetry
) {
    lateinit var drivetrain: SwerveDriveTrain
    lateinit var intake: IntakeSubsystem
    lateinit var deposit: DepositSubsystem

    /**
     * I2C
     */
    private val imuLock = Any()
    @Volatile var imu: BNO055IMU? = null
    private var imuThread: Thread? = null
    @Volatile var imuAngle = 0.0
    @Volatile var imuOffset: Double = 0.0
    @Volatile var voltage: Double = 0.0
    private var voltageTimer: ElapsedTime? = null
    private val startingIMUOffset = 0.0

    private lateinit var allHubs: List<LynxModule>
    private lateinit var controlHub: LynxModule

    fun init() {
        instance = this

        if (Env.USING_IMU) {
            synchronized(imuLock) {
                imu = hardwareMap.get(BNO055IMU::class.java, "imu")
                imu?.let { safeImu ->
                    val parameters = BNO055IMU.Parameters()
                    parameters.angleUnit = BNO055IMU.AngleUnit.RADIANS
                    safeImu.initialize(parameters)
                }
            }
        }

        this.telemetry = MultipleTelemetry(telemetry, FtcDashboard.getInstance().telemetry)

        voltageTimer = ElapsedTime()

        allHubs = hardwareMap.getAll(LynxModule::class.java)

        allHubs.forEach { hub ->
            hub.bulkCachingMode = LynxModule.BulkCachingMode.MANUAL
            if (hub.isParent && LynxConstants.isEmbeddedSerialNumber(hub.serialNumber)) {
                controlHub = hub
            }
        }

        voltage = hardwareMap.voltageSensor.iterator().next().voltage

    }

    fun write() {
        try {
            drivetrain.write()
        } catch (_: Exception) {}

        deposit.write()
    }

    fun loop(pose: Pose) {
        try {
            drivetrain.drive(pose)
            drivetrain.updateModules()
        } catch (_: Exception) {}

        deposit.periodic()

        if (voltageTimer!!.seconds() > 5) {
            voltageTimer!!.reset()
            voltage = hardwareMap.voltageSensor.iterator().next().voltage
        }
    }

    fun read() {
        try {
            drivetrain.read()
        } catch (_: Exception) {}

        deposit.read()
    }

    fun clearBulkCache() {
        controlHub.clearBulkCache()
    }

    fun startIMUThread(opMode: LinearOpMode) {
        imuThread = Thread {
            while (!opMode.isStopRequested && opMode.opModeIsActive()) {
                synchronized(imuLock) {
                    imuAngle = (((imu!!.angularOrientation?.firstAngle
                        ?: (0.0 + startingIMUOffset))).toDouble())
                }
            }
        }
        imuThread!!.start()
    }

    companion object {
        lateinit var instance: OutreachBot
            private set
    }
}