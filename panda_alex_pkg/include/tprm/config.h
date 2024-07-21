/*********************************************************************
 *
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2022,
 *  ETH Zurich - V4RL, Department of Mechanical and Process Engineering.
 *  All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above
 *     copyright notice, this list of conditions and the following
 *     disclaimer in the documentation and/or other materials provided
 *     with the distribution.
 *   * Neither the name of the institute nor the names of its
 *     contributors may be used to endorse or promote products derived
 *     from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 *  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 *  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 *  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 *  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 *  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.
 *
 * Authors: Matthias Busenhart
 *********************************************************************/

#ifndef __TPRM_CONFIG_H__
#define __TPRM_CONFIG_H__

#include <Eigen/Core>
#include <iostream>
#include <vector>

namespace tprm {
/**
 * @brief Type definition for a vector of Eigen::Vector3d. Used everywhere.
 */
using Vector3d = Eigen::Vector3d;

/**
 * @brief Parameters for the Robot
 * 
 * This class contains all the parameters for the holonomic robot assumed in the planning problem.
 */
class HolonomicRobot {
public:
    /**
     * @brief Movement speed in all directions. [m/s]
     * 
     * Default value: 0.5 m/s
     */
    static double movement_speed;
};

struct Vector7d {
    
    Eigen::VectorXd data;
    
    Vector7d()
    {
        Vector7d(0);
    }

    Vector7d(double a)
    {
        std::vector<double> vect = {a,a,a,a,a,a,a};
        data = Eigen::Map<Eigen::VectorXd, Eigen::Unaligned>(vect.data(), vect.size());
    }

    Vector7d(std::vector<double> vect)
    {
        data = Eigen::Map<Eigen::VectorXd, Eigen::Unaligned>(vect.data(), vect.size());
    }

    Vector7d(Eigen::VectorXd vect)
    {
        data = vect;
    }

    Vector7d(const Vector7d& vect)
    {
        this->data = vect.data;
    }

    Vector7d(double a, double b, double c, double d, double e, double f, double g)
    {
        std::vector<double> vect = {a,b,c,d,e,f,g};
        data = Eigen::Map<Eigen::VectorXd, Eigen::Unaligned>(vect.data(), vect.size());
    }

    double operator()(int index) const
    {
        return this->data(index);
    }

    void operator()(int index, double value)
    {
        this->data(index) = value;
    }

    Vector7d operator-(Vector7d rhs)
    {
        Vector7d result(0);
        for (int i = 0; i < 7; i++)
        {
            result.data(i) = this->data(i) - rhs.data(i);
        }

        return result;
    }

    Vector7d operator-(const Vector7d rhs) const
    {
        Vector7d result(0);
        for (int i = 0; i < 7; i++)
        {
            result.data(i) = this->data(i) - rhs.data(i);
        }

        return result;
    }

    Vector7d operator+(const Vector7d rhs)
    {
        Vector7d result(0);
        for (int i = 0; i < 7; i++)
        {
            result.data(i) = this->data(i) + rhs.data(i);
        }

        return result;
    }

    Vector7d operator*(const double value)
    {
        Vector7d result(0);
        for (int i = 0; i < 7; i++)
        {
            result.data(i) = this->data(i) * value;
        }

        return result;
    }

    Vector7d operator/(const double value)
    {
        Vector7d result(0);
        for (int i = 0; i < 7; i++)
        {
            result.data(i) = this->data(i) / value;
        }

        return result;
    }

    void operator=(const Vector7d& rhs)
    {
        for (int i = 0; i < 7; i++)
        {
            (*this)(i,rhs(i));
        }
    }

    static std::string toString(const Vector7d& rhs)
    {
        std::string res;
        for (int i = 0; i < 7; i++)
        {
            res += std::to_string(rhs.data(i));
            res += " ";
        }
        return res;
    }

    Vector7d cwiseProduct(const Vector7d rhs)
    {
        Vector7d result = Vector7d(0);
        auto res = this->data.cwiseProduct(rhs.data);

        for (int i = 0; i < 7; i++)
        {
            result(i, res(i));
        }

        return result;
    }

    Eigen::VectorXd transpose()
    {
        return this->data.transpose();
    }

    double squaredNorm()
    {
        return this->data.squaredNorm();
    }

    double norm()
    {
        return this->data.norm();
    }

    std::vector<double> convertToStdVect() const
    {
        std::vector<double> result;
        for (int i = 0 ; i < 7; i++)
        {
            result.push_back((*this)(i));
        }

        return result;
    }

    static double randomValue()
    {
        double res = (float)(rand()) / (float)(RAND_MAX) * 6.28 - 3.14;

        return res;
    }

    static Vector7d Random()
    {
        Vector7d result(0);
        for (int i = 0; i < 7; i++)
        {
            result(i, randomValue());
        }

        return result;
    }
};

} /* namespace tprm */

#endif /* __TPRM_CONFIG_H__ */