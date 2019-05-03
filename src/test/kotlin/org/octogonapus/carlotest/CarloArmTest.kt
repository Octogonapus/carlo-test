package org.octogonapus.carlotest

import arrow.core.extensions.either.monad.binding
import arrow.core.flatMap
import arrow.core.getOrHandle
import com.google.common.collect.ImmutableList
import com.google.inject.Injector
import com.neuronrobotics.bowlerkernel.hardware.Script
import com.neuronrobotics.bowlerkernel.hardware.device.BowlerDevice
import com.neuronrobotics.bowlerkernel.hardware.device.BowlerDeviceFactory
import com.neuronrobotics.bowlerkernel.hardware.device.deviceid.DefaultConnectionMethods
import com.neuronrobotics.bowlerkernel.hardware.device.deviceid.DefaultDeviceTypes
import com.neuronrobotics.bowlerkernel.hardware.device.deviceid.DeviceId
import com.neuronrobotics.bowlerkernel.hardware.deviceresource.provisioned.DigitalState
import com.neuronrobotics.bowlerkernel.hardware.deviceresource.resourceid.DefaultAttachmentPoints
import com.neuronrobotics.bowlerkernel.hardware.deviceresource.unprovisioned.UnprovisionedDigitalOutFactory
import com.neuronrobotics.bowlerkernel.hardware.deviceresource.unprovisioned.UnprovisionedServoFactory
import com.neuronrobotics.bowlerkernel.kinematics.base.DefaultKinematicBase
import com.neuronrobotics.bowlerkernel.kinematics.base.baseid.SimpleKinematicBaseId
import com.neuronrobotics.bowlerkernel.kinematics.closedloop.NoopBodyController
import com.neuronrobotics.bowlerkernel.kinematics.limb.DefaultLimb
import com.neuronrobotics.bowlerkernel.kinematics.limb.limbid.SimpleLimbId
import com.neuronrobotics.bowlerkernel.kinematics.limb.link.DefaultLink
import com.neuronrobotics.bowlerkernel.kinematics.limb.link.DhParam
import com.neuronrobotics.bowlerkernel.kinematics.limb.link.Link
import com.neuronrobotics.bowlerkernel.kinematics.limb.link.LinkType
import com.neuronrobotics.bowlerkernel.kinematics.limb.link.toFrameTransformation
import com.neuronrobotics.bowlerkernel.kinematics.motion.ForwardKinematicsSolver
import com.neuronrobotics.bowlerkernel.kinematics.motion.FrameTransformation
import com.neuronrobotics.bowlerkernel.kinematics.motion.InverseKinematicsSolver
import com.neuronrobotics.bowlerkernel.kinematics.motion.NoopInertialStateEstimator
import com.neuronrobotics.bowlerkernel.kinematics.motion.plan.DefaultLimbMotionPlanFollower
import com.neuronrobotics.bowlerkernel.util.Limits
import org.jlleitschuh.guice.getInstance
import org.junit.jupiter.api.Test
import org.junit.jupiter.api.fail
import org.octogonapus.ktguava.collections.immutableListOf
import org.octogonapus.ktguava.collections.immutableMapOf
import org.octogonapus.ktguava.collections.toImmutableList
import java.net.InetAddress

internal class CarloArmTest {

    private val links: ImmutableList<Link> = immutableListOf(
        DefaultLink(
            LinkType.Rotary,
            DhParam(44.45, 0, 196.85, 0),
            Limits(180, -180),
            NoopInertialStateEstimator
        ),
        DefaultLink(
            LinkType.Rotary,
            DhParam(38.1, 0, 273.05, 0),
            Limits(180, -180),
            NoopInertialStateEstimator
        )
    )

    private val dhParams = links.map { it.dhParam }.toImmutableList()

    private val limb1Id = SimpleLimbId("carlo-arm-limb1")

    private val fkEngine = object : ForwardKinematicsSolver {
        override fun solveChain(currentJointAngles: ImmutableList<Double>) =
            dhParams.mapIndexed { index, dhParam ->
                dhParam.copy(theta = currentJointAngles[index] + dhParam.theta)
            }.toFrameTransformation()
    }

    private val ikEngine = object : InverseKinematicsSolver {
        override fun solveChain(
            currentJointAngles: ImmutableList<Double>,
            targetFrameTransform: FrameTransformation
        ): ImmutableList<Double> {
            // TODO: Real IK
            return currentJointAngles.mapIndexed { index, _ -> index.toDouble() + 90 }.toImmutableList()
        }
    }

    @Test
    fun `test homing with real arm`() {
        val injector = Script.makeScriptInjector().createChildInjector(Script.getDefaultModules())

        val device = getDevice(injector)

        val servoFactory = injector.getInstance<UnprovisionedServoFactory>()
        val servos = links.mapIndexed { index, _ ->
            servoFactory.makeUnprovisionedServo(
                device,
                DefaultAttachmentPoints.Pin((32 + index).toByte())
            ).flatMap { device.add(it) }.getOrHandle { fail(it) }
        }.toImmutableList()

        val limb1 = DefaultLimb(
            limb1Id,
            links,
            fkEngine,
            ikEngine,
            CarloLimbMotionPlanGenerator(ikEngine),
            DefaultLimbMotionPlanFollower(),
            links.mapIndexed { index, _ ->
                ServoJointAngleController(servos[index])
            }.toImmutableList(),
            NoopInertialStateEstimator
        )

        val base = DefaultKinematicBase(
            SimpleKinematicBaseId("carlo-arm-base"),
            immutableListOf(limb1),
            immutableMapOf(limb1Id to FrameTransformation.identity),
            NoopBodyController
        )

        val carlo = CarloArm(base)
        carlo.homeLimbs(200)

        device.disconnect().mapLeft { fail(it) }
    }

    @Test
    fun `test homing with fake arm`() {
        val limb1 = DefaultLimb(
            limb1Id,
            links,
            fkEngine,
            ikEngine,
            CarloLimbMotionPlanGenerator(ikEngine),
            DefaultLimbMotionPlanFollower(),
            links.mapIndexed { index, _ ->
                SimulatedJointAngleController("$index")
            }.toImmutableList(),
            NoopInertialStateEstimator
        )

        val base = DefaultKinematicBase(
            SimpleKinematicBaseId("carlo-arm-base"),
            immutableListOf(limb1),
            immutableMapOf(limb1Id to FrameTransformation.identity),
            NoopBodyController
        )

        val carlo = CarloArm(base)
        carlo.homeLimbs(200)
    }

    @Test
    fun `test blinking led`() {
        val injector = Script.makeScriptInjector().createChildInjector(Script.getDefaultModules())

        val device = getDevice(injector)

        val digitalOutFactory = injector.getInstance<UnprovisionedDigitalOutFactory>()
        val led1 = digitalOutFactory.makeUnprovisionedDigitalOut(
            device,
            DefaultAttachmentPoints.Pin(32)
        ).flatMap { device.add(it) }.getOrHandle { fail(it) }

        repeat(100) {
            val state = if (it % 2 == 0) DigitalState.HIGH else DigitalState.LOW
            led1.write(state)
            Thread.sleep(16)
        }

        device.disconnect().getOrHandle { fail(it) }
    }
}

private fun getDevice(injector: Injector) = binding<String, BowlerDevice> {
    val (device) = injector.getInstance<BowlerDeviceFactory>().makeBowlerDevice(
        DeviceId(
            DefaultDeviceTypes.Esp32wroom32,
            DefaultConnectionMethods.InternetAddress(
                InetAddress.getByAddress(
                    listOf(192, 168, 4, 1).map { it.toByte() }.toByteArray()
                )
            )
        )
    )
    device.connect()
    device
}.getOrHandle { fail(it) }
