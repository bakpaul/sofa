/******************************************************************************
*                 SOFA, Simulation Open-Framework Architecture                *
*                    (c) 2006 INRIA, USTL, UJF, CNRS, MGH                     *
*                                                                             *
* This program is free software; you can redistribute it and/or modify it     *
* under the terms of the GNU Lesser General Public License as published by    *
* the Free Software Foundation; either version 2.1 of the License, or (at     *
* your option) any later version.                                             *
*                                                                             *
* This program is distributed in the hope that it will be useful, but WITHOUT *
* ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or       *
* FITNESS FOR A PARTICULAR PURPOSE. See the GNU Lesser General Public License *
* for more details.                                                           *
*                                                                             *
* You should have received a copy of the GNU Lesser General Public License    *
* along with this program. If not, see <http://www.gnu.org/licenses/>.        *
*******************************************************************************
* Authors: The SOFA Team and external contributors (see Authors.txt)          *
*                                                                             *
* Contact information: contact@sofa-framework.org                             *
******************************************************************************/
#ifndef SOFA_HELPER_SYSTEM_THREAD_CTIME_H
#define SOFA_HELPER_SYSTEM_THREAD_CTIME_H

#include <ctime>
#include <sofa/helper/config.h>

#ifdef WIN32
# include <windows.h>
#endif


/********************
 * Time measurement *
 ********************/


namespace sofa::helper::system::thread
{

#ifdef WIN32
typedef ULONGLONG ctime_t;
#else
typedef unsigned long long ctime_t;
#endif

class SOFA_HELPER_API CTime
{
public:
    //volatile causes: "warning: type qualifiers ignored on function return type" on GCC 4.3

    // Get current reference time
    static /*volatile*/ ctime_t getRefTime();

    // Get the frequency of the reference timer
    static ctime_t getRefTicksPerSec();

    // Get current time using the fastest available method
    static /*volatile*/ ctime_t getFastTime();

    // Get the frequency of the fast timer
    static ctime_t getTicksPerSec();

    // Same as getFastTime, but with the additional guaranty that it will never decrease.
    static /*volatile*/ ctime_t getTime();

    // Sleep for the given duration in second
    static void sleep(double s);

    static double toSecond(const ctime_t t) ;

protected:
    static ctime_t computeTicksPerSec();
};

} // namespace sofa::helper::system::thread


#endif
