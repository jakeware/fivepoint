// This file is part of FivePoint
// 
// Copyright (C) 2016 Vincent Lui (vincent.lui@monash.edu), Monash University
// 
// This program is free software: you can redistribute it and/or modify
// it under the terms of the GNU General Public License as published by
// the Free Software Foundation, either version 3 of the License, or
// (at your option) any later version.
// 
// This program is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
// GNU General Public License for more details.
// 
// You should have received a copy of the GNU General Public License
// along with this program.  If not, see <http://www.gnu.org/licenses/>.

#ifndef LEVELHELPERS_H
#define LEVELHELPERS_H

#include <TooN/TooN.h>

namespace lvlhelpers
{
inline double
LevelScale(const float fScale,
           const int nLevel)
{
    return std::pow(fScale,nLevel);
}

inline double
LevelZeroPos(const double dLevelPos,
             const float fScale,
             const int nLevel)
{
    return dLevelPos * LevelScale(fScale,nLevel);
}

inline TooN::Vector<2>
LevelZeroPos(const TooN::Vector<2> v2LevelPos,
             const float fScale,
             const int nLevel)
{
    TooN::Vector<2> v2;
    v2[0] = LevelZeroPos(v2LevelPos[0],fScale,nLevel);
    v2[1] = LevelZeroPos(v2LevelPos[1],fScale,nLevel);
    return v2;
}

inline double
LevelNPos(const double dRootPos,
          const float fScale,
          const int nLevel)
{
    return dRootPos / LevelScale(fScale,nLevel);
}

inline TooN::Vector<2>
LevelNPos(const TooN::Vector<2> v2RootPos,
          const float fScale,
          const int nLevel)
{
    TooN::Vector<2> v2;
    v2[0] = LevelNPos(v2RootPos[0],fScale,nLevel);
    v2[1] = LevelNPos(v2RootPos[1],fScale,nLevel);
    return v2;
}
}

#endif // LEVELHELPERS_H
