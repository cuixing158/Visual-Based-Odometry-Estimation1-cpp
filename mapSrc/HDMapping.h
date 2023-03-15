#ifndef HDMAPPING_H
#define HDMAPPING_H

#include "constructWorldMap_types.h"
#include "constructWorldMap_types1.h"
#include "rtwtypes.h"
#include "omp.h"
#include <cstddef>
#include <cstdlib>

namespace buildMapping {
class HDMapping {
public:
  HDMapping();
  ~HDMapping();
  void b_constructWorldMap(const struct0_T *inputArgs,
                           struct1_T *inputOutputStruct);
  constructWorldMapStackData *getStackData();

private:
  constructWorldMapPersistentData pd_;
  constructWorldMapStackData SD_;
};

} // namespace buildMapping

namespace buildMapping {
extern omp_nest_lock_t constructWorldMap_nestLockGlobal;
}

#endif
