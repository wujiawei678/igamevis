#pragma once

#include <iGameFilter.h>
#include <iGameSurfaceMesh.h>
#include <iGameUnstructuredMesh.h>
#include <vector>

IGAME_NAMESPACE_BEGIN

class ValidateCellsFilter : public Filter {
public:
    I_OBJECT(ValidateCellsFilter);
    static Pointer New() { return new ValidateCellsFilter; }

    bool Execute() override;

    const std::vector<igIndex>& GetInvalidCellIds() const { return m_InvalidCellIds; }
    int GetInvalidCellCount() const { return static_cast<int>(m_InvalidCellIds.size()); }

protected:
    ValidateCellsFilter();
    ~ValidateCellsFilter() override = default;

private:
    bool ValidateUnstructuredMesh(UnstructuredMesh::Pointer mesh);
    bool ValidateSurfaceMesh(SurfaceMesh::Pointer mesh);
    bool IsTriangleValid(const Point& p0, const Point& p1, const Point& p2,
                        double minArea, double minEdge) const;
    bool IsTetraValid(const Point& p0, const Point& p1, const Point& p2,
                      const Point& p3, double minVolume, double minEdge) const;
    bool IsPolygonValid(const std::vector<Point>& points,
                        double minArea, double minEdge) const;

    std::vector<igIndex> m_InvalidCellIds;
};

IGAME_NAMESPACE_END
