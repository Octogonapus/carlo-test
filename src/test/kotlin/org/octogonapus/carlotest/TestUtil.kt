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

import com.google.common.collect.ImmutableList
import com.neuronrobotics.bowlerkernel.kinematics.limb.link.DefaultLink
import com.neuronrobotics.bowlerkernel.kinematics.limb.link.DhParam
import com.neuronrobotics.bowlerkernel.kinematics.limb.link.Link
import com.neuronrobotics.bowlerkernel.kinematics.limb.link.LinkType
import com.neuronrobotics.bowlerkernel.kinematics.motion.NoopInertialStateEstimator
import com.neuronrobotics.bowlerkernel.util.Limits
import org.octogonapus.ktguava.collections.immutableListOf

internal val vex15Links: ImmutableList<Link> = immutableListOf(
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

internal val seaArmLinks: ImmutableList<Link> = immutableListOf(
    DefaultLink(
        LinkType.Rotary,
        DhParam(135, 0, 0, -90),
        Limits(180, -180),
        NoopInertialStateEstimator
    ),
    DefaultLink(
        LinkType.Rotary,
        DhParam(0, 0, 175, 0),
        Limits(180, -180),
        NoopInertialStateEstimator
    ),
    DefaultLink(
        LinkType.Rotary,
        DhParam(0, 90, 169.28, 0),
        Limits(180, -180),
        NoopInertialStateEstimator
    )
)
