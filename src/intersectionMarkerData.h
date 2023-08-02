/**
    Copyright (c) 2023 Takayoshi Matsumoto
    You may use, distribute, or modify this code under the terms of the MIT license.
*/
#pragma once

#include <vector>
#include <maya/MUserData.h>
#include <maya/MPointArray.h>
#include <maya/MVectorArray.h>

class IntersectionMarkerData : public MUserData
{
public:

#if MAYA_API_VERSION < 20220000
     IntersectionMarkerData() : MUserData(/* deleteAfterUse */ false) {}
#else
     IntersectionMarkerData() : MUserData() {}
#endif
    ~IntersectionMarkerData() {}

    struct FaceData {
        MPointArray vertices;
        MPointArray edges;
        MVectorArray normals;
    };

    std::vector<FaceData> faces;
};
