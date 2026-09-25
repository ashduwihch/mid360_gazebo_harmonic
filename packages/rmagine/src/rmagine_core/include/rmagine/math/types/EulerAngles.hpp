/*
 * Copyright (c) 2024, University Osnabrück
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *     * Redistributions of source code must retain the above copyright
 *       notice, this list of conditions and the following disclaimer.
 *     * Redistributions in binary form must reproduce the above copyright
 *       notice, this list of conditions and the following disclaimer in the
 *       documentation and/or other materials provided with the distribution.
 *     * Neither the name of the University Osnabrück nor the
 *       names of its contributors may be used to endorse or promote products
 *       derived from this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
 * ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
 * WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 * DISCLAIMED. IN NO EVENT SHALL University Osnabrück BE LIABLE FOR ANY
 * DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
 * (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 * LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
 * ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 * (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
 * SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

/**
 * @file
 * 
 * @brief Euler Angles
 *
 * @date 03.10.2024
 * @author Alexander Mock
 * 
 * @copyright Copyright (c) 2024, University Osnabrück. All rights reserved.
 * This project is released under the 3-Clause BSD License.
 * 
 */
#ifndef RMAGINE_MATH_EULER_ANGLES_HPP
#define RMAGINE_MATH_EULER_ANGLES_HPP

#include "definitions.h"
#include <rmagine/types/shared_functions.h>
#include <initializer_list>
#include <iostream>

namespace rmagine
{

/**
 * @brief EulerAngles type
 * 
 */
template<typename DataT>
struct EulerAngles_ 
{
    // DATA
    DataT roll;     // x-axis
    DataT pitch;    // y-axis
    DataT yaw;      // z-axis


    // Functions
    RMAGINE_FUNCTION
    static EulerAngles_<DataT> Identity()
    {
        EulerAngles_<DataT> ret;
        ret.setIdentity();
        return ret;
    }

    RMAGINE_INLINE_FUNCTION
    void setIdentity();

    RMAGINE_INLINE_FUNCTION
    void set(const Quaternion_<DataT>& q);

    RMAGINE_INLINE_FUNCTION
    void set(const Matrix_<DataT, 3, 3>& M);

    RMAGINE_INLINE_FUNCTION
    Vector3_<DataT> mult(const Vector3_<DataT>& v) const;

    //////////////////
    // Operators

    RMAGINE_INLINE_FUNCTION
    Vector3_<DataT> operator*(const Vector3_<DataT>& v) const 
    {
        return mult(v);
    }

    ///////////////
    // CASTING

    /**
     * @brief EulerAngles -> Quaternion
     * 
     * @return Quaternion_<DataT> 
     */
    RMAGINE_INLINE_FUNCTION
    operator Quaternion_<DataT>() const;

    /**
     * @brief EulerAngles -> Rotation Matrix
     * 
     * @return Matrix_<DataT, 3, 3> 
     */
    RMAGINE_INLINE_FUNCTION
    operator Matrix_<DataT, 3, 3>() const;

    /**
     * @brief Data Type Cast to ConvT
     * 
     * @tparam ConvT 
     */
    template<typename ConvT>
    RMAGINE_INLINE_FUNCTION
    EulerAngles_<ConvT> cast() const;
};

} // rmagine

#include "EulerAngles.tcc"

#endif // RMAGINE_MATH_EULER_ANGLES_HPPs