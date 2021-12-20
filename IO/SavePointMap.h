#pragma once

#include <RVC/RVC.h>

extern int SavePlyFile(const char *save_address, RVC::PointMap &pm, const bool saveNormal = false);

extern int SavePlyFile(const char *save_address, const double *xyzs, const unsigned int nPts);