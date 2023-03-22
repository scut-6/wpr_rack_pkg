#pragma once
#include "RackDetect.h"

class CStorageRack : public CRackDetect
{
public:
    CStorageRack();
    ~CStorageRack();
    void GenTemp(bool inScale);
};