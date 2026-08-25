#include "MyFilter/iGameValidateCellsFilter.h"

#include <cmath>
#include <limits>

IGAME_NAMESPACE_BEGIN

namespace {
// Use relative thresholds based on mesh size
constexpr double kEdgeScale = 1e-8;       // min edge = diag * kEdgeScale
constexpr double kAreaScale = 1e-12;      // min area = diag^2 * kAreaScale
constexpr double kVolumeScale = 1e-15;    // min volume = diag^3 * kVolumeScale
}

ValidateCellsFilter::ValidateCellsFilter() {
    SetNumberOfInputs(1);
    SetNumberOfOutputs(1);
}

bool ValidateCellsFilter::Execute() {
    DataObject::Pointer input = GetInput(0);
    if (input == nullptr) {
        return false;
    }

    m_InvalidCellIds.clear();

    if (auto mesh = DynamicCast<UnstructuredMesh>(input)) {
        if (!ValidateUnstructuredMesh(mesh)) {
            return false;
        }
    } else if (auto mesh = DynamicCast<SurfaceMesh>(input)) {
        if (!ValidateSurfaceMesh(mesh)) {
            return false;
        }
    } else {
        return false;
    }

    SetOutput(0, input);

    if (m_Model != nullptr && !m_InvalidCellIds.empty()) {
        auto selection = m_Model->GetSelection();
        if (selection != nullptr) {
            selection->SelectionCallBackEvent(IG_CELL, m_InvalidCellIds, Selection::Operate::Add);
        }
    }

    return true;
}

bool ValidateCellsFilter::ValidateUnstructuredMesh(UnstructuredMesh::Pointer mesh) {
    if (mesh == nullptr) {
        return false;
    }

    // derive thresholds from mesh bounding box
    double diag = mesh->GetBoundingBox().diag();
    if (diag <= 0.0) diag = 1.0;
    double minEdge = diag * kEdgeScale;
    double minArea = diag * diag * kAreaScale;
    double minVolume = diag * diag * diag * kVolumeScale;

    const auto cellCount = mesh->GetNumberOfCells();

    // reuse containers to reduce allocations
    IdArray::Pointer ids = IdArray::New();
    std::vector<Point> points;

    for (IGsize cellId = 0; cellId < cellCount; ++cellId) {
        ids->Reset();
        mesh->GetCellPointIds(cellId, ids);
        int nPts = ids->GetNumberOfIds();
        points.clear();
        points.reserve(static_cast<size_t>(std::max(0, nPts)));
        for (int i = 0; i < nPts; ++i) {
            points.push_back(mesh->GetPoint(ids->GetId(i)));
        }

        // cell type may give semantic meaning to point count
        int cellType = mesh->GetCellType(cellId);
        bool valid = true;

        if (nPts <= 0) {
            valid = false;
        } else if (nPts == 1) {
            valid = true; // single-vertex cell considered valid
        } else if (nPts == 2) {
            double e = (points[0] - points[1]).norm();
            valid = e > minEdge;
        } else if (cellType == IG_TRIANGLE || nPts == 3) {
            valid = IsTriangleValid(points[0], points[1], points[2], minArea, minEdge);
        } else if (cellType == IG_TETRA || nPts == 4) {
            if (nPts < 4) {
                valid = false;
            } else {
                valid = IsTetraValid(points[0], points[1], points[2], points[3], minVolume, minEdge);
            }
        } else {
            // general polygon / polyhedron face: treat as polygon in 3D (compute area via triangulation around centroid)
            valid = IsPolygonValid(points, minArea, minEdge);
        }

        if (!valid) {
            m_InvalidCellIds.push_back(static_cast<igIndex>(cellId));
        }
    }

    return true;
}

bool ValidateCellsFilter::ValidateSurfaceMesh(SurfaceMesh::Pointer mesh) {
    if (mesh == nullptr) {
        return false;
    }

    double diag = mesh->GetBoundingBox().diag();
    if (diag <= 0.0) diag = 1.0;
    double minEdge = diag * kEdgeScale;
    double minArea = diag * diag * kAreaScale;

    const auto faceCount = mesh->GetNumberOfFaces();
    std::vector<Point> points;
    points.reserve(3);
    for (IGsize faceId = 0; faceId < faceCount; ++faceId) {
        igIndex pointIds[3]{};
        mesh->GetFacePointIds(faceId, pointIds);

        points.clear();
        for (int i = 0; i < 3; ++i) {
            points.push_back(mesh->GetPoint(pointIds[i]));
        }

        if (!IsTriangleValid(points[0], points[1], points[2], minArea, minEdge)) {
            m_InvalidCellIds.push_back(static_cast<igIndex>(faceId));
        }
    }

    return true;
}

bool ValidateCellsFilter::IsTriangleValid(const Point& p0, const Point& p1, const Point& p2, double minArea, double minEdge) const {
    Vector3f e1 = p1 - p0;
    Vector3f e2 = p2 - p0;
    Vector3f crossP = CrossProduct(e1, e2);
    double area = 0.5 * static_cast<double>(crossP.norm());
    if (area <= minArea) return false;

    double l0 = (p1 - p0).norm();
    double l1 = (p2 - p1).norm();
    double l2 = (p0 - p2).norm();
    if (l0 <= minEdge || l1 <= minEdge || l2 <= minEdge) return false;
    return true;
}

bool ValidateCellsFilter::IsTetraValid(const Point& p0, const Point& p1, const Point& p2, const Point& p3, double minVolume, double minEdge) const {
    Vector3f v1 = p1 - p0;
    Vector3f v2 = p2 - p0;
    Vector3f v3 = p3 - p0;
    double vol6 = std::abs(DotProduct(v1, CrossProduct(v2, v3)));
    double volume = vol6 / 6.0; // actual tetra volume
    if (volume <= minVolume) return false;

    // check edges
    double minE = std::numeric_limits<double>::max();
    const Point arr[4] = {p0, p1, p2, p3};
    for (int i = 0; i < 4; ++i) {
        for (int j = i + 1; j < 4; ++j) {
            double e = (arr[i] - arr[j]).norm();
            if (e < minE) minE = e;
        }
    }
    if (minE <= minEdge) return false;
    return true;
}

bool ValidateCellsFilter::IsPolygonValid(const std::vector<Point>& points, double minArea, double minEdge) const {
    int n = static_cast<int>(points.size());
    if (n < 3) return false;

    // compute centroid
    Vector3f center{0.0f, 0.0f, 0.0f};
    for (const auto& p : points) center += p;
    center /= static_cast<float>(n);

    // triangulate around centroid and accumulate area
    double area = 0.0;
    for (int i = 0; i < n; ++i) {
        const auto& a = points[i];
        const auto& b = points[(i + 1) % n];
        Vector3f va = a - center;
        Vector3f vb = b - center;
        area += 0.5 * static_cast<double>(CrossProduct(va, vb).norm());
    }
    if (area <= minArea) return false;

    // check minimal edge length
    double minE = std::numeric_limits<double>::max();
    for (int i = 0; i < n; ++i) {
        double e = (points[i] - points[(i + 1) % n]).norm();
        if (e < minE) minE = e;
    }
    if (minE <= minEdge) return false;

    return true;
}
IGAME_NAMESPACE_END
