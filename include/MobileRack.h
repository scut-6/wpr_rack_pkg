#pragma once
#include "RackDetect.h"

class CMobileRack : public CRackDetect
{
public:
    CMobileRack();
    ~CMobileRack();
    void GenTemp(bool inScale);
};