#include "KDTreeKernel.h"

#include <maya/MStatus.h>
#include <maya/MBoundingBox.h>
#include <maya/MMatrix.h>

#include <vector>


std::vector<TriangleData> KDTreeKernel::extractTriangles(
    const MObject& meshObject,
    const MMatrix& offsetMatrix
    
) const {
    std::vector<TriangleData> triangles;

    MItMeshPolygon itPoly(meshObject);
    for(; !itPoly.isDone(); itPoly.next()) {

        int numTriangles;
        itPoly.numTriangles(numTriangles);

        for (int triangleId=0; triangleId < numTriangles; ++triangleId) {
            MPointArray points;
            MIntArray vertexList;
            itPoly.getTriangle(triangleId, points, vertexList, MSpace::kObject);
            TriangleData triangle(
                    itPoly.index(),
                    triangleId,
                    points[0] * offsetMatrix,
                    points[1] * offsetMatrix,
                    points[2] * offsetMatrix);

            triangles.push_back(triangle);
        }
    }
    
    return triangles;
}


MStatus KDTreeKernel::build(

    const MObject& meshObject,
    const MBoundingBox& bbox,
    const MMatrix& offsetMatrix

) {
    // 1.
    std::vector<TriangleData> triangles = extractTriangles(meshObject, offsetMatrix);
    if (triangles.empty()) {
        return MStatus::kFailure;
    }

    // 2.
    root = new KDTreeNode;
    root->boundingBox = bbox;

    // 3.
    for (const auto& triangle : triangles) {
        insertTriangle(root, triangle);
    }

    return MStatus::kSuccess;
}

void KDTreeKernel::insertTriangle(KDTreeNode* node, const TriangleData& triangle) {

    if(!intersectBoxBox(node->boundingBox, triangle.bbox)) {
        return;
    }

    if (node->isLeaf()) {
        node->triangles.push_back(triangle);
        if (node->triangles.size() > maxTrianglesPerLeaf) {
            splitNode(node);
        }

        return;

    } else {  // not leaf

        if (triangle.bbox.center()[node->splitDimension] < node->splitValue) {
            insertTriangle(node->left, triangle);
        } else {
            insertTriangle(node->right, triangle);
        }
    }

}


void KDTreeKernel::splitNode(KDTreeNode* node) {

    // Select the longest axis as the split axis
    double xExtent = node->boundingBox.width();
    double yExtent = node->boundingBox.height();
    double zExtent = node->boundingBox.depth();

    if (xExtent > yExtent && xExtent > zExtent) {
        node->splitDimension = 0;
    } else if (yExtent > zExtent) {
        node->splitDimension = 1;
    } else {
        node->splitDimension = 2;
    }

    node->splitValue = (
        node->boundingBox.min()[node->splitDimension] +
        node->boundingBox.max()[node->splitDimension]
    ) * 0.5;

    node->left = new KDTreeNode;
    node->right = new KDTreeNode;

    setChildBoundingBoxes(node);

    // Move triangles to child nodes
    for (const auto& triangle : node->triangles) {
        if (triangle.center()[node->splitDimension] < node->splitValue) {
            node->left->triangles.push_back(triangle);
        } else {
            node->right->triangles.push_back(triangle);
        }
    }

    node->triangles.clear();
}


void KDTreeKernel::setChildBoundingBoxes(KDTreeNode* node) {

    MPoint lMin = node->boundingBox.min();
    MPoint lMax = node->boundingBox.max();
    MPoint rMin = node->boundingBox.min();
    MPoint rMax = node->boundingBox.max();

    lMax[node->splitDimension] = node->splitValue;
    rMin[node->splitDimension] = node->splitValue;

    node->left->boundingBox  = MBoundingBox(lMin, lMax);
    node->right->boundingBox = MBoundingBox(rMin, rMax);
}


std::vector<TriangleData> queryIntersectedRecursive(
        const TriangleData& triangle,
        const KDTreeNode* node
) {
    std::vector<TriangleData> intersectedTriangles;

    if (node->isLeaf()) {
        for (const auto& nodeTriangle : node->triangles) {
            if (intersectTriangleTriangle(triangle, nodeTriangle)) {
                intersectedTriangles.push_back(nodeTriangle);
            }
        }
    } else {
        if (intersectBoxBox(node->boundingBox, triangle.bbox)) {
            std::vector<TriangleData> leftTriangles = queryIntersectedRecursive(triangle, node->left);
            std::vector<TriangleData> rightTriangles = queryIntersectedRecursive(triangle, node->right);
            intersectedTriangles.insert(
                    intersectedTriangles.end(),
                    leftTriangles.begin(),
                    leftTriangles.end());
            intersectedTriangles.insert(
                    intersectedTriangles.end(),
                    rightTriangles.begin(),
                    rightTriangles.end());
        }
    }
    /*  TODO: profile this and see if it's faster
        if (triangleIntersectsBoundingBox(triangle, node->left->boundingBox)) {
            std::vector<TriangleData> leftResult = queryIntersectedRecursive(node->left, triangle);
            result.insert(result.end(), leftResult.begin(), leftResult.end());
        }
        if (triangleIntersectsBoundingBox(triangle, node->right->boundingBox)) {
            std::vector<TriangleData> rightResult = queryIntersectedRecursive(node->right, triangle);
            result.insert(result.end(), rightResult.begin(), rightResult.end());
        }
    }
    */

    return intersectedTriangles;
}


std::vector<TriangleData> KDTreeKernel::queryIntersected(
        const TriangleData& triangle
) const {
    return queryIntersectedRecursive(triangle, root);
}


void KDTreeKernel::clear(KDTreeNode* node) {
    if (node->left) {
        clear(node->left);
    }
    if (node->right) {
        clear(node->right);
    }
    delete node;
}
