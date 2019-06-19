/**
 * File: FORB.h
 * Date: June 2012
 * Author: Dorian Galvez-Lopez
 * Description: functions for ORB descriptors
 * License: see the LICENSE.txt file
 *
 */

#ifndef __D_T_F_FREAK__
#define __D_T_F_FREAK__

#include <opencv2/core/core.hpp>
#include <vector>
#include <string>

#include "FClass.h"

namespace DBoW2 {

/// Functions to manipulate ORB descriptors
class FFREAK: protected FORB
{
public:
    using FORB::TDescriptor;
    using FORB::pDescriptor;
    using FORB::L;
    using FORB::distance;
    using FORB::meanValue;
    using FORB::toString;
    using FORB::fromString;
    using FORB::fromArray;
    using FORB::toMat32F;
    using FORB::toMat8U;
    using FORB::toArray8U;
    using FORB::fromArray8U;
};

} // namespace DBoW2

#endif

