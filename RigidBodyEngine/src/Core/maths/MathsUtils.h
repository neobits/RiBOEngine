//
//  MathUtils.h
//  RiBo Engine
//
//  Created by Angelo Moro on 10/12/2019
//

#pragma once

#include <math.h>
#include <iostream>
#include "Vector.h"
#include "Quaternion.h"
#include "Matrix.h"

#define EPSILON 0.001f

namespace MathUtils
{
	static tkQuat& VecMultQuat(const tkVec3& v_lhs, const tkQuat& q_rhs)
	{
		tkQuat q(v_lhs.x, v_lhs.y, v_lhs.z, 0);
		return q*q_rhs;
	}
}