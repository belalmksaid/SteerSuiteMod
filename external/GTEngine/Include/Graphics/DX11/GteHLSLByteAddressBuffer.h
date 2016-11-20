// David Eberly, Geometric Tools, Redmond WA 98052
// Copyright (c) 1998-2016
// Distributed under the Boost Software License, Version 1.0.
// http://www.boost.org/LICENSE_1_0.txt
// http://www.geometrictools.com/License/Boost/LICENSE_1_0.txt
// File Version: 2.1.0 (2016/01/25)

#pragma once

#include <Graphics/DX11/GteHLSLResource.h>

namespace gte
{

class GTE_IMPEXP HLSLByteAddressBuffer : public HLSLResource
{
public:
    // Construction and destruction.
    virtual ~HLSLByteAddressBuffer();

    HLSLByteAddressBuffer(D3D11_SHADER_INPUT_BIND_DESC const& desc);

    HLSLByteAddressBuffer(D3D11_SHADER_INPUT_BIND_DESC const& desc,
        unsigned int index);

    // Member access.
    bool IsGpuWritable() const;

private:
    bool mGpuWritable;
};

}
