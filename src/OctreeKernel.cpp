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


MStatus OctreeKernel::build(const MObject& meshObject, const MBoundingBox& bbox)
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
            // TODO: Improve this by pointing to the same memory location
            // triangle.vertices[i] = itPoly.getTriangle(i, points, MSpace::kWorld);
            itPoly.getTriangle(i, points, vertexList, MSpace::kWorld);
            triangle.vertices[0] = points[0];
            triangle.vertices[1] = points[1];
            triangle.vertices[2] = points[2];
            triangle.faceIndex = itPoly.index();

            // Add the triangle to the octree
            insertTriangle(root, triangle);
        }
    }

    return MStatus::kSuccess;
}


int MAX_TRIANGLES_PER_NODE = 10;

void OctreeKernel::insertTriangle(OctreeNode* node, const TriangleData& triangle)
{
    if (node->isLeaf()) {
        if (node->triangles.size() < MAX_TRIANGLES_PER_NODE) {
            // If this leaf node can still hold more triangles, add it here
            node->triangles.push_back(triangle);
        } else {
            // If this leaf node is full, split it and then try to add the triangle again
            splitNode(node);
            insertTriangle(node, triangle);
        }
    } else {
        // If this is not a leaf node, try to add the triangle to its children
        bool inserted = false;
        for (int i = 0; i < 8; ++i) {
            if (node->children[i] != nullptr && boxTriangleIntersect(node->children[i]->boundingBox, triangle, false)) {
                insertTriangle(node->children[i], triangle);
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
            if (boxTriangleIntersect(boxes[i], triangle, false)) {
                node->children[i]->triangles.push_back(triangle);
            }
        }
    }

    // Clear triangles in current node
    node->triangles.clear();
}


std::vector<TriangleData> OctreeKernel::queryIntersected(const TriangleData& triangle) const
{
    std::vector<TriangleData> intersectedTriangles;
    std::queue<OctreeNode*> nodesToCheck;
    
    if (root != nullptr) {
        nodesToCheck.push(root);
    }

    while (!nodesToCheck.empty()) {
        OctreeNode* currentNode = nodesToCheck.front();
        nodesToCheck.pop();

        if (boxTriangleIntersect(currentNode->boundingBox, triangle, true)) {
            if (currentNode->isLeaf()) {
                // If the current node is a leaf and its bounding box intersects with the triangle,
                // then all its triangles are considered as intersected triangles
                std::copy(
                    currentNode->triangles.begin(),
                    currentNode->triangles.end(),
                    std::back_inserter(intersectedTriangles)
                );
            } else {
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


bool OctreeKernel::boxTriangleIntersect(const MBoundingBox& box, const TriangleData& triangle, bool preciseMode) const
{
    for (const MPoint& vertex : triangle.vertices) {
        if (box.contains(vertex)) {
            return true;
        }
    }

    if (!preciseMode) {
        return false;
    }

    // Create a bounding box for the triangle
    MBoundingBox triangleBox(triangle.vertices[0], triangle.vertices[0]);
    triangleBox.expand(triangle.vertices[1]);
    triangleBox.expand(triangle.vertices[2]);

    // Check if the triangle's bounding box intersects with the given box
    if (box.intersects(triangleBox)) {
        return true;
    }

    // Note: This function might fail to detect intersections where the triangle
    // penetrates the box but none of the triangle's vertices are contained within the box.
    return false;

}
