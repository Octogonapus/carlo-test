package org.octogonapus.carlotest

import com.neuronrobotics.bowlerkernel.kinematics.base.KinematicBase
import com.neuronrobotics.bowlerkernel.kinematics.limb.link.toFrameTransformation
import com.neuronrobotics.bowlerkernel.kinematics.motion.BasicMotionConstraints

class CarloArm(private val base: KinematicBase) {

    private val relaxedMotionConstraints = BasicMotionConstraints(0, 10, 100, 100)

    /**
     * Homes all limbs.
     *
     * @param time The duration of the homing movement.
     */
    fun homeLimbs(time: Number) {
        base.limbs.forEach { limb ->
            val homePosition = limb.links.map { it.dhParam }.toFrameTransformation()
            limb.setDesiredTaskSpaceTransform(
                homePosition,
                relaxedMotionConstraints.copy(motionDuration = time)
            )
        }

        // Wait for limbs to stop moving
        while (allLimbsMoving()) {
            Thread.sleep(1)
        }
    }

    private fun allLimbsMoving() =
        base.limbs.map { it.isMovingToTaskSpaceTransform() }.reduce { acc, elem -> acc && elem }
}
