// SPDX-License-Identifier: LGPL-2.1-or-later

#include <gtest/gtest.h>

#include <algorithm>
#include <array>
#include <cmath>
#include <numbers>
#include <set>

#include "Mod/Sketcher/App/planegcs/GCS.h"

class SystemTest: public GCS::System
{
public:
    size_t getNumberOfConstraints(int tagID = -1)
    {
        return _getNumberOfConstraints(tagID);
    }
};

class GCSTest: public ::testing::Test
{
protected:
    void SetUp() override
    {
        _system = std::make_unique<SystemTest>();
    }

    void TearDown() override
    {
        _system.reset();
    }

    SystemTest* System()
    {
        return _system.get();
    }

private:
    std::unique_ptr<SystemTest> _system;
};

TEST_F(GCSTest, clearConstraints)  // NOLINT
{
    // Arrange
    const size_t numConstraints {100};
    for (size_t i = 0; i < numConstraints; ++i) {
        System()->addConstraint(new GCS::Constraint());
    }
    ASSERT_EQ(numConstraints, System()->getNumberOfConstraints());

    // Act
    System()->clear();

    // Assert
    EXPECT_EQ(0, System()->getNumberOfConstraints());
}

// Regression guard for the drag-Jacobian column bug. A permanent Equal
// constraint lives in the priority subsystem (subsysA), whose Jacobian the
// two-subsystem SQP drag solver assembles via calcJacobi(plistAB, ..) with a
// params array (plistAB) that differs in size and order from the subsystem's
// own plist. The previous Equal fast path cached its output column keyed to
// plist, so during dragging the Equal gradient landed in the wrong column and
// the constraint was not enforced. After the fix the two params stay equal
// while the drag pulls them to the target.
TEST(DragSolveTest, equalConstraintEnforcedDuringDrag)  // NOLINT
{
    GCS::System sys;
    double x0 = 1.0, y0 = 1.0;
    double x1 = 1.0, y1 = 0.0;

    GCS::Point p0(&x0, &y0), p1(&x1, &y1);

    // Permanent constraint (tag >= 0): x0 == x1. Routes to subSystems (subsysA).
    sys.addConstraintEqual(&x0, &x1);

    GCS::VEC_pD params = {&x0, &y0, &x1, &y1};
    sys.declareUnknowns(params);
    sys.initSolution();
    ASSERT_EQ(sys.solve(true, GCS::DogLeg), GCS::Success);
    sys.applySolution();

    // Drag point (MoveParameters, external to the declared unknowns) coincident
    // with p0, tag = -1 → subSystemsAux (subsysB). Its param set differs from
    // subsysA's, so plistAB is a reordered superset of subsysA's plist.
    double drag_x = x0, drag_y = y0;
    GCS::Point dragP(&drag_x, &drag_y);
    sys.addConstraintP2PCoincident(dragP, p0, GCS::DefaultTemporaryConstraint);
    sys.initSolution();

    // Move the drag target; the SQP solve must pull p0 toward it while keeping
    // the permanent Equal (x0 == x1) satisfied.
    drag_x = 7.0;
    drag_y = 3.0;
    ASSERT_EQ(sys.solve(true, GCS::DogLeg), GCS::Success);
    sys.applySolution();

    EXPECT_NEAR(x0, x1, 1e-6) << "Equal constraint (x0==x1) not enforced during drag "
                              << "(x0=" << x0 << ", x1=" << x1 << ")";
    EXPECT_NEAR(x0, 7.0, 1e-6) << "drag did not pull p0 to target x (x0=" << x0 << ")";
    EXPECT_NEAR(y0, 3.0, 1e-6) << "drag did not pull p0 to target y (y0=" << y0 << ")";
}

// Regression guard for the sparse DogLeg's rank-deficiency handling. Redundant
// (duplicate) constraints make the least-norm normal matrix J*J^T singular. Because
// SimplicialLDLT::info() only reports EXACT zero pivots, a near-singular factorization
// can slip through and produce a NaN/Inf step; the solver must detect the non-finite
// step and fall back to a dense solve, still converging on the (consistent) system.
TEST(SparseDogLegTest, redundantConstraintsDoNotProduceNaN)  // NOLINT
{
    GCS::System sys;
    double x0 = 0.0, y0 = 0.0;
    double x1 = 3.0, y1 = 0.0;
    double d = 5.0;

    GCS::Point p0(&x0, &y0), p1(&x1, &y1);

    // Two identical distance constraints on the same point pair -> J has duplicate
    // rows (rank 1), so with 4 free params (under-constrained) J*J^T (2x2) is singular.
    sys.addConstraintP2PDistance(p0, p1, &d);
    sys.addConstraintP2PDistance(p0, p1, &d);

    GCS::VEC_pD params = {&x0, &y0, &x1, &y1};
    sys.declareUnknowns(params);
    sys.initSolution();

    int res = sys.solve(true, GCS::DogLeg);
    sys.applySolution();

    // The result must be finite and satisfy the (consistent) distance constraint.
    EXPECT_TRUE(std::isfinite(x0) && std::isfinite(y0) && std::isfinite(x1) && std::isfinite(y1))
        << "solve produced a non-finite result on a rank-deficient system";
    double dist = std::hypot(x1 - x0, y1 - y0);
    EXPECT_NEAR(dist, 5.0, 1e-6)
        << "distance constraint not satisfied (dist=" << dist << ", res=" << res << ")";
}

// Component-decomposed diagnose(): differential tests against the monolithic path.
// The two paths must report identical dofs, conflicting/redundant tags and
// dependent-parameter sets for multi-component systems.
namespace
{

// Three independent segments; the second carries a duplicated (redundant)
// distance, the third a conflicting pair of distances. First point of each
// segment is fixed (not declared unknown), second is free.
struct DiagnoseFixture
{
    // storage lives here so pointers stay valid for the System's lifetime
    std::array<double, 12> coords;
    std::array<double, 5> dims;
    GCS::VEC_pD unknowns;

    void build(GCS::System& sys)
    {
        coords = {0, 0, 3, 4,       // segment 1
                  20, 0, 23, 4,     // segment 2
                  40, 0, 43, 4};    // segment 3
        dims = {5.0, 5.0, 5.0, 5.0, 7.0};

        GCS::Point p1a(&coords[0], &coords[1]), p1b(&coords[2], &coords[3]);
        GCS::Point p2a(&coords[4], &coords[5]), p2b(&coords[6], &coords[7]);
        GCS::Point p3a(&coords[8], &coords[9]), p3b(&coords[10], &coords[11]);

        sys.addConstraintP2PDistance(p1a, p1b, &dims[0], 1);
        sys.addConstraintP2PDistance(p2a, p2b, &dims[1], 2);
        sys.addConstraintP2PDistance(p2a, p2b, &dims[2], 3);  // redundant with tag 2
        sys.addConstraintP2PDistance(p3a, p3b, &dims[3], 4);
        sys.addConstraintP2PDistance(p3a, p3b, &dims[4], 5);  // conflicts with tag 4

        unknowns = {&coords[2], &coords[3], &coords[6], &coords[7], &coords[10], &coords[11]};
        sys.declareUnknowns(unknowns);
    }
};

}  // namespace

TEST(DiagnoseComponentTest, multiComponentMatchesMonolithic)  // NOLINT
{
    DiagnoseFixture fixMono;
    GCS::System sysMono;
    fixMono.build(sysMono);
    sysMono.useComponentDiagnose = false;
    int dofsMono = sysMono.diagnose();

    DiagnoseFixture fixComp;
    GCS::System sysComp;
    fixComp.build(sysComp);
    sysComp.useComponentDiagnose = true;
    int dofsComp = sysComp.diagnose();

    EXPECT_EQ(dofsMono, dofsComp);

    GCS::VEC_I confMono, confComp, redMono, redComp, partMono, partComp;
    sysMono.getConflicting(confMono);
    sysComp.getConflicting(confComp);
    sysMono.getRedundant(redMono);
    sysComp.getRedundant(redComp);
    sysMono.getPartiallyRedundant(partMono);
    sysComp.getPartiallyRedundant(partComp);

    EXPECT_EQ(confMono, confComp);
    EXPECT_EQ(redMono, redComp);
    EXPECT_EQ(partMono, partComp);

    // dependent parameters must be positionally identical sets; compare via the
    // index each parameter has in the respective system's unknown list
    auto depIndices = [](const GCS::System& sys, const GCS::VEC_pD& unknowns) {
        GCS::VEC_pD dep;
        sys.getDependentParams(dep);
        std::set<size_t> indices;
        for (double* p : dep) {
            auto it = std::find(unknowns.begin(), unknowns.end(), p);
            EXPECT_NE(it, unknowns.end());
            indices.insert(static_cast<size_t>(it - unknowns.begin()));
        }
        return indices;
    };
    EXPECT_EQ(depIndices(sysMono, fixMono.unknowns), depIndices(sysComp, fixComp.unknowns));

    // sanity on the semantics themselves, not only the differential:
    // 6 unknowns minus one independent distance per segment = 3 DoF
    EXPECT_EQ(dofsComp, 3);
    // tag 5 conflicts (or is reported against tag 4); tags 2/3 hold a redundancy
    EXPECT_FALSE(confComp.empty());
    EXPECT_FALSE(redComp.empty());
}

TEST(DiagnoseComponentTest, fullyConstrainedComponentsMatchMonolithic)  // NOLINT
{
    // Two segments, each fully pinned: CoordinateX/Y on the free endpoint plus a
    // consistent redundant distance -> over-/redundant handling across components.
    auto build = [](GCS::System& sys, std::array<double, 8>& c, std::array<double, 8>& d) {
        c = {0, 0, 3, 4, 20, 0, 23, 4};
        d = {3, 4, 5, 0, 23, 4, 5, 0};
        GCS::Point p1b(&c[2], &c[3]);
        GCS::Point p2b(&c[6], &c[7]);
        sys.addConstraintCoordinateX(p1b, &d[0], 1);
        sys.addConstraintCoordinateY(p1b, &d[1], 2);
        GCS::Point p1a(&c[0], &c[1]);
        sys.addConstraintP2PDistance(p1a, p1b, &d[2], 3);  // consistent redundant
        sys.addConstraintCoordinateX(p2b, &d[4], 4);
        sys.addConstraintCoordinateY(p2b, &d[5], 5);
        GCS::VEC_pD unknowns = {&c[2], &c[3], &c[6], &c[7]};
        sys.declareUnknowns(unknowns);
    };

    std::array<double, 8> cMono, dMono, cComp, dComp;
    GCS::System sysMono, sysComp;
    build(sysMono, cMono, dMono);
    build(sysComp, cComp, dComp);
    sysMono.useComponentDiagnose = false;
    sysComp.useComponentDiagnose = true;

    int dofsMono = sysMono.diagnose();
    int dofsComp = sysComp.diagnose();

    EXPECT_EQ(dofsMono, dofsComp);
    EXPECT_EQ(dofsComp, 0);  // fully constrained, redundancy is consistent

    GCS::VEC_I confMono, confComp, redMono, redComp;
    sysMono.getConflicting(confMono);
    sysComp.getConflicting(confComp);
    sysMono.getRedundant(redMono);
    sysComp.getRedundant(redComp);
    EXPECT_EQ(confMono, confComp);
    EXPECT_EQ(redMono, redComp);
    EXPECT_FALSE(redComp.empty());  // the distance is redundant
}
