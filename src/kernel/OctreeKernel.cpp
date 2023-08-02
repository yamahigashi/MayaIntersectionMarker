#include "OctreeKernel.h"

#include <maya/MMatrix.h>
#include <maya/MFnMesh.h>
#include <maya/MFnDagNode.h>
#include <maya/MItMeshPolygon.h>
#include <maya/MBoundingBox.h>
#include <maya/MPoint.h>
#include <maya/MPointArray.h>
#include <maya/MIntArray.h>
#include <maya/MGlobal.h>
#include <maya/MStatus.h>

#include <queue>


MStatus OctreeKernel::build(const MObject& meshObject, const MBoundingBox& bbox, const MMatrix& offsetMatrix)
{
    MStatus status;
    // Clear previous data if exists
    if (root != nullptr) {
        clear(root);
    }

    root = new OctreeNode;
    root->boundingBox = bbox;

    // Iterate over all polygons in the mesh
    TriangleData triangle;
    MItMeshPolygon itPoly(meshObject);
    // MGlobal::displayInfo("Building octree...");
    for (; !itPoly.isDone(); itPoly.next()) {

        int numTriangles;
        itPoly.numTriangles(numTriangles);

        for (int i = 0; i < numTriangles; ++i) {

            MPointArray points;
            MIntArray vertexList;
            itPoly.getTriangle(i, points, vertexList, MSpace::kObject);

            MPoint p0 = points[0] * offsetMatrix;
            MPoint p1 = points[1] * offsetMatrix;
            MPoint p2 = points[2] * offsetMatrix;

            TriangleData triangle(itPoly.index(), i, p0, p1, p2);
            // Add the triangle to the octree
            insertTriangle(root, triangle, 0);
        }
    }

    return MStatus::kSuccess;
}


int MAX_TRIANGLES_PER_NODE = 10;
int MAX_DEPTH = 32;

void OctreeKernel::insertTriangle(OctreeNode* node, const TriangleData& triangle, int depth)
{
    if (depth > MAX_DEPTH) {
        // MGlobal::displayError("Octree depth exceeded!");

        // If we have reached the maximum depth, add the triangle to this node
        node->triangles.push_back(triangle);
        return;
    }

    if (node->isLeaf()) {
        if (node->triangles.size() < MAX_TRIANGLES_PER_NODE) {
            // If this leaf node can still hold more triangles, add it here
            node->triangles.push_back(triangle);
        } else {
            // If this leaf node is full, split it and then try to add the triangle again
            splitNode(node);
            insertTriangle(node, triangle , depth + 1);
        }
    } else {
        // If this is not a leaf node, try to add the triangle to its children
        bool inserted = false;
        for (int i = 0; i < 8; ++i) {
            if (node->children[i] != nullptr && boxContainsAnyVertices(node->children[i]->boundingBox, triangle)) {
                insertTriangle(node->children[i], triangle , depth + 1);
                inserted = true;
            }
        }

        // If the triangle was not added to any of the children, add it to this node
        if (!inserted) {
            node->triangles.push_back(triangle);
        }
    }
}


void OctreeKernel::splitNode(OctreeNode* node)
{
    // Calculate new bounding boxes for child nodes
    MPoint center = node->boundingBox.center();
    MPoint min = node->boundingBox.min();
    MPoint max = node->boundingBox.max();
    
    // Create new bounding boxes
    MBoundingBox boxes[8];
    boxes[0] = MBoundingBox(min, center);
    boxes[1] = MBoundingBox(MPoint(center.x, min.y, min.z), MPoint(max.x, center.y, center.z));
    boxes[2] = MBoundingBox(MPoint(center.x, min.y, center.z), MPoint(max.x, center.y, max.z));
    boxes[3] = MBoundingBox(MPoint(min.x, min.y, center.z), MPoint(center.x, center.y, max.z));
    boxes[4] = MBoundingBox(MPoint(min.x, center.y, min.z), MPoint(center.x, max.y, center.z));
    boxes[5] = MBoundingBox(MPoint(center.x, center.y, min.z), MPoint(max.x, max.y, center.z));
    boxes[6] = MBoundingBox(center, max);
    boxes[7] = MBoundingBox(MPoint(min.x, center.y, center.z), MPoint(center.x, max.y, max.z));

    // Create child nodes
    for (int i = 0; i < 8; ++i) {
        node->children[i] = new OctreeNode();
        node->children[i]->boundingBox = boxes[i];
    }

    // Move triangles to child nodes
    for (const TriangleData& triangle : node->triangles) {
        for (int i = 0; i < 8; ++i) {
            if (boxContainsAnyVertices(boxes[i], triangle)) {
                node->children[i]->triangles.push_back(triangle);
            }
        }
    }

    // Clear triangles in current node
    node->triangles.clear();
}


std::vector<TriangleData> OctreeKernel::queryIntersected(const TriangleData& incomingTri) const
{
    std::vector<TriangleData> intersectedTriangles;
    std::queue<OctreeNode*> nodesToCheck;
    
    if (root != nullptr) {
        nodesToCheck.push(root);
    }

    while (!nodesToCheck.empty()) {
        OctreeNode* currentNode = nodesToCheck.front();
        nodesToCheck.pop();

        if (intersectBoxTriangle(currentNode->boundingBox, incomingTri)) {

            if (currentNode->isLeaf()) {
                for (const TriangleData& ourTri: currentNode->triangles) {
                    if (intersectTriangleTriangle(ourTri, incomingTri)) {
                        intersectedTriangles.push_back(ourTri);
                    }
                }
            } else {
                // If the current node is a leaf and its bounding box intersects with the triangle,
                // If the current node is not a leaf, then check its children
                for (int i = 0; i < 8; ++i) {
                    if (currentNode->children[i] != nullptr) {
                        nodesToCheck.push(currentNode->children[i]);
                    }
                }
            }
        }
    }

    return intersectedTriangles;
}


void OctreeKernel::clear(OctreeNode* node)
{
    if (node != nullptr) {
        // Recursively delete child nodes
        for (int i = 0; i < 8; ++i) {
            if (node->children[i] != nullptr) {
                clear(node->children[i]);
            }
        }

        // Delete the node itself
        delete node;
    }
}


K2KIntersection OctreeKernel::intersectKernelKernel(
    SpatialDivisionKernel& otherKernel
) const {

    std::vector<TriangleData> intersectedTriangles;
    std::vector<TriangleData> otherIntersectedTriangles;

    OctreeKernel* other = dynamic_cast<OctreeKernel*>(&otherKernel);
    if (other == nullptr) {
        MGlobal::displayError("Cannot intersect octree with other kernel type!");
        return std::make_pair(intersectedTriangles, otherIntersectedTriangles);
    }


    std::queue<std::pair<OctreeNode*, OctreeNode*>> nodesToCheck;

    if (this->root != nullptr && other->root != nullptr) {
        nodesToCheck.push(std::make_pair(this->root, other->root));
    }

    while (!nodesToCheck.empty()) {
        std::pair<OctreeNode*, OctreeNode*> currentNodes = nodesToCheck.front();
        nodesToCheck.pop();

        if (currentNodes.first->boundingBox.intersects(currentNodes.second->boundingBox)) {
            if (currentNodes.first->isLeaf() && currentNodes.second->isLeaf()) {
                // If both nodes are leaves and their bounding boxes intersect,
                // then all their triangles are considered as intersected triangles
                std::copy(
                    currentNodes.first->triangles.begin(),
                    currentNodes.first->triangles.end(),
                    std::back_inserter(intersectedTriangles)
                );
                std::copy(
                    currentNodes.second->triangles.begin(),
                    currentNodes.second->triangles.end(),
                    std::back_inserter(otherIntersectedTriangles)
                );
            } else {
                // If one of the nodes is not a leaf, then check its children
                for (int i = 0; i < 8; ++i) {
                    if (currentNodes.first->children[i] != nullptr && currentNodes.second->children[i] != nullptr) {
                        nodesToCheck.push(std::make_pair(currentNodes.first->children[i], currentNodes.second->children[i]));
                    }
                }
            }
        }
    }

    return std::make_pair(intersectedTriangles, otherIntersectedTriangles);
}
