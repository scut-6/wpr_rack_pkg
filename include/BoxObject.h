#pragma once
#include "RackDetect.h"

class CBoxObject : public CRackDetect
{
public:
    CBoxObject();
    ~CBoxObject();
    void GenTemp(bool inScale);
};