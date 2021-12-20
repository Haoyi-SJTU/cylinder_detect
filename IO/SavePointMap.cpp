#include "SavePointMap.h"

#include <fstream>
#include <iostream>

int SavePlyFile(const char *save_address, RVC::PointMap &pm, const bool saveNormal) {
    if (!pm.IsValid()) {
        std::cout << "point map is invalid!" << std::endl;
        return 1;
    }
    std::ofstream file;
    double *normal_data = saveNormal ? pm.GetNormalDataPtr() : nullptr;
    file.open(save_address);
    file << "ply" << std::endl;
    file << "format ascii 1.0" << std::endl;
    file << "comment Created by Rvbust, Inc" << std::endl;
    RVC::Size pm_size = pm.GetSize();
    int pm_num = pm_size.rows * pm_size.cols;
    file << "element vertex " << pm_num << std::endl;
    file << "property float x" << std::endl;
    file << "property float y" << std::endl;
    file << "property float z" << std::endl;
    if (normal_data != nullptr) {
        file << "property float nx" << std::endl;
        file << "property float ny" << std::endl;
        file << "property float nz" << std::endl;
    }
    file << "end_header" << std::endl;
    double *pm_data = pm.GetPointDataPtr();
    for (int i = 0; i < pm_num; i++) {
        file << (float)*pm_data << " " << (float)*(pm_data + 1) << " " << (float)*(pm_data + 2);
        if (normal_data != nullptr) {
            file << " " << (float)*normal_data << " " << (float)*(normal_data + 1) << " " << (float)*(normal_data + 2);
            normal_data += 3;
        }
        file << std::endl;
        pm_data = pm_data + 3;
    }
    file.close();

    return 0;
}

int SavePlyFile(const char *save_address, const double *xyzs, const unsigned int nPts) {
    std::ofstream file;
    file.open(save_address);
    file << "ply" << std::endl;
    file << "format ascii 1.0" << std::endl;
    file << "comment Created by Rvbust, Inc" << std::endl;
    int pm_num = nPts;
    file << "element vertex " << pm_num << std::endl;
    file << "property float x" << std::endl;
    file << "property float y" << std::endl;
    file << "property float z" << std::endl;
    file << "end_header" << std::endl;
    const double *pm_data = xyzs;
    for (int i = 0; i < pm_num; i++) {
        file << (float)*pm_data << " " << (float)*(pm_data + 1) << " " << (float)*(pm_data + 2) << std::endl;
        pm_data = pm_data + 3;
    }
    file.close();

    return 0;
}