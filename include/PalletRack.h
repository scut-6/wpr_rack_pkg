#pragma once
#include "RackDetect.h"

class CPalletRack : public CRackDetect
{
public:
    CPalletRack();
    ~CPalletRack();
    void GenTemp(bool inScale);
};