/**
    Copyright (c) 2023 Takayoshi Matsumoto
    You may use, distribute, or modify this code under the terms of the MIT license.
*/
#pragma once

#include "SpatialDivisionKernel.h"

#include <string>
#include <vector>
#include <list>
#include <utility> // for std::pair
#include <unordered_set>
#include <unordered_map>

#include <maya/MDagPath.h>
#include <maya/MDataHandle.h>
#include <maya/MFnDependencyNode.h>
#include <maya/MPxNode.h>
#include <maya/MPxLocatorNode.h>
#include <maya/MObject.h>
#include <maya/MPlug.h>
#include <maya/MString.h>
#include <maya/MTypeId.h>

#define MESH_A             "inMeshA"
#define MESH_B             "inMeshB"
#define OFFSET_MATRIX_A    "offsetMatrixA"
#define OFFSET_MATRIX_B    "offsetMatrixB"
#define REST_INTERSECTED   "restIntersected"
#define VERTEX_CHECKSUM_A  "vertexChecksumA"
#define VERTEX_CHECKSUM_B  "vertexChecksumB"

#define KERNEL             "kernel"
#define COLLISION_MODE     "collisionMode"
#define OUTPUT_INTERSECTED "outputIntersected"
#define OUT_MESH           "outMesh"
#define CACHE_SIZE         10000


struct pair_hash {
    template <class T1, class T2>
    std::size_t operator () (const std::pair<T1, T2>& p) const {
        auto h1 = std::hash<T1>{}(p.first);
        auto h2 = std::hash<T2>{}(p.second); 

        // Mainly for demonstration purposes, i.e. works but is overly simple
        // In the real world, use sth. like boost.hash_combine
        return h1 ^ h2;  
    }
};



// The type for the value of the cache
struct CacheResultType {
    std::unordered_set<int> first;
    std::unordered_set<int> second;
};



template <typename CacheKeyType, typename CacheValueType, typename HashFunc>
class LRUCache {
private:
    size_t max_size_;
    std::list<std::pair<CacheKeyType, CacheValueType>> list_;
    std::unordered_map<CacheKeyType, typename std::list<std::pair<CacheKeyType, CacheValueType>>::iterator, HashFunc> map_;

public:
    LRUCache(size_t max_size) : max_size_(max_size) {}

    void put(const CacheKeyType& key, const CacheValueType& value)
    {
        auto iter = map_.find(key);
        if (iter != map_.end())
        {
            // Update the existing value and move it to the front of the list
            iter->second->second = value;
            list_.splice(list_.begin(), list_, iter->second);
        }
        else
        {
            // Add a new value to the cache
            list_.emplace_front(key, value);
            map_[key] = list_.begin();

            // If the cache is too large, remove the least recently used item
            if (list_.size() > max_size_) {
                map_.erase(list_.back().first);
                list_.pop_back();
            }
        }
    }

    CacheValueType get(const CacheKeyType& key)
    {
        auto iter = map_.find(key);
        if (iter != map_.end())
        {
            // Move the accessed item to the front of the list
            list_.splice(list_.begin(), list_, iter->second);
            return iter->second->second;
        }
        else
        {
            throw std::out_of_range("Key not found in cache");
        }
    }
};

using CacheKeyType = std::pair<int, int>;
using CacheType = LRUCache<CacheKeyType, CacheResultType, pair_hash>;


class IntersectionMarkerNode : public MPxLocatorNode
{
public:
                        IntersectionMarkerNode();
                        ~IntersectionMarkerNode() override;

    static  void*       creator();
    static  MStatus     initialize();

            MStatus     postEvaluation(const  MDGContext& context, const MEvaluationNode& evaluationNode, PostEvaluationType evalType) override;
            MStatus     compute(const MPlug &plug, MDataBlock &dataBlock);

    // static MStatus      onInitializePlugin();
    // static MStatus      onUninitializePlugin();

    static MStatus      getCacheKey(MObject &node, std::string &key);
    static MStatus      getCacheKeyFromMesh(MObject &meshObjA, MObject &meshObjB, std::string &key);

std::shared_ptr<SpatialDivisionKernel> getActiveKernel() const;
            MStatus     checkIntersections(MObject &meshAObject, MObject &meshBObject, std::shared_ptr<SpatialDivisionKernel> kernel, MMatrix offset);
            MStatus     preEvaluation(const MDGContext& context, const MEvaluationNode& evaluationNode) override;
            MStatus     getInputDagMesh(const MObject inputAttr, MFnMesh &outMesh) const;
            MStatus     getOffsetMatrix(const MObject inputAttr, MMatrix &outMatrix) const;
            MStatus     getChecksumA(int &outChecksum) const;
            MStatus     getChecksumB(int &outChecksum) const;
      MBoundingBox      getBoundingBox(const MObject &meshObject) const;
           MStatus      createMeshFromTriangles(const MObject& meshAObject, const MIntArray& intersectedTriangleIDs, MFnMesh& outputMeshFn);

             MPlug      meshAPlug() const { return MPlug(thisMObject(), meshA); }
             MPlug      meshBPlug() const { return MPlug(thisMObject(), meshB); }

public:
    static MObject      meshA;
    static MObject      meshB;
    static MObject      offsetMatrixA;
    static MObject      offsetMatrixB;
    static MObject      restIntersected;

    static MObject      vertexChecksumA;
    static MObject      vertexChecksumB;
    static MObject      kernelType;
    static MObject      collisionMode;

    static MObject      outputIntersected;
    
    static MString      NODE_NAME;
    static MTypeId      NODE_ID;

    static MString      drawDbClassification;
    static MString      drawRegistrantId;

  static CacheType      cache;
    std::unordered_set<int> intersectedFaceIdsA;
    std::unordered_set<int> intersectedFaceIdsB;
};
