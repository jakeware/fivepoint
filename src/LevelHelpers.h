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
