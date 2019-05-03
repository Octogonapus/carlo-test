/*
 * This file is part of carlo-test.
 *
 * carlo-test is free software: you can redistribute it and/or modify
 * it under the terms of the GNU Lesser General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * carlo-test is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU Lesser General Public License for more details.
 *
 * You should have received a copy of the GNU Lesser General Public License
 * along with carlo-test.  If not, see <https://www.gnu.org/licenses/>.
 */
package org.octogonapus.carlotest

import com.neuronrobotics.bowlerkernel.kinematics.closedloop.JointAngleController
import com.neuronrobotics.bowlerkernel.kinematics.motion.MotionConstraints

class SimulatedJointAngleController(
    private val name: String
) : JointAngleController {

    private var angle = 10.0

    override fun getCurrentAngle() = angle.also {
        println("SJAC $name: get $angle")
    }

    override fun setTargetAngle(angle: Double, motionConstraints: MotionConstraints) {
        this.angle = angle
        println("SJAC $name: set $angle")
    }
}
