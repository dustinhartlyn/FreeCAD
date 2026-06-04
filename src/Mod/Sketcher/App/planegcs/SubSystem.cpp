// SPDX-License-Identifier: LGPL-2.1-or-later

/***************************************************************************
 *   Copyright (c) 2011 Konstantinos Poulios <logari81@gmail.com>          *
 *                                                                         *
 *   This file is part of the FreeCAD CAx development system.              *
 *                                                                         *
 *   This library is free software; you can redistribute it and/or         *
 *   modify it under the terms of the GNU Library General Public           *
 *   License as published by the Free Software Foundation; either          *
 *   version 2 of the License, or (at your option) any later version.      *
 *                                                                         *
 *   This library  is distributed in the hope that it will be useful,      *
 *   but WITHOUT ANY WARRANTY; without even the implied warranty of        *
 *   MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the         *
 *   GNU Library General Public License for more details.                  *
 *                                                                         *
 *   You should have received a copy of the GNU Library General Public     *
 *   License along with this library; see the file COPYING.LIB. If not,    *
 *   write to the Free Software Foundation, Inc., 59 Temple Place,         *
 *   Suite 330, Boston, MA  02111-1307, USA                                *
 *                                                                         *
 ***************************************************************************/

#ifdef _MSC_VER
# pragma warning(disable : 4251)
#endif

#include <iostream>
#include <iterator>
#include <unordered_map>

#include "SubSystem.h"


namespace GCS
{

// SubSystem
SubSystem::SubSystem(std::vector<Constraint*>& clist_, VEC_pD& params)
    : clist(clist_)
{
    MAP_pD_pD dummymap;
    initialize(params, dummymap);
}

SubSystem::SubSystem(std::vector<Constraint*>& clist_, VEC_pD& params, MAP_pD_pD& reductionmap)
    : clist(clist_)
{
    initialize(params, reductionmap);
}

SubSystem::~SubSystem()
{}

void SubSystem::initialize(VEC_pD& params, MAP_pD_pD& reductionmap)
{
    csize = static_cast<int>(clist.size());

    // tmpplist will contain the subset of parameters from params that are
    // relevant for the constraints listed in clist
    VEC_pD tmpplist;
    {
        SET_pD s1(params.begin(), params.end());
        SET_pD s2;
        for (std::vector<Constraint*>::iterator constr = clist.begin(); constr != clist.end();
             ++constr) {
            (*constr)->revertParams();  // ensure that the constraint points to the original parameters
            VEC_pD constr_params = (*constr)->params();
            s2.insert(constr_params.begin(), constr_params.end());
        }
        std::set_intersection(s1.begin(), s1.end(), s2.begin(), s2.end(), std::back_inserter(tmpplist));
    }

    plist.clear();
    MAP_pD_I rindex;
    if (!reductionmap.empty()) {
        int i = 0;
        MAP_pD_I pindex;
        for (VEC_pD::const_iterator itt = tmpplist.begin(); itt != tmpplist.end(); ++itt) {
            MAP_pD_pD::const_iterator itr = reductionmap.find(*itt);
            if (itr != reductionmap.end()) {
                MAP_pD_I::const_iterator itp = pindex.find(itr->second);
                if (itp == pindex.end()) {  // the reduction target is not in plist yet, so add it now
                    plist.push_back(itr->second);
                    rindex[itr->first] = i;
                    pindex[itr->second] = i;
                    i++;
                }
                else {  // the reduction target is already in plist, just inform rindex
                    rindex[itr->first] = itp->second;
                }
            }
            else if (pindex.find(*itt) == pindex.end()) {  // not in plist yet, so add it now
                plist.push_back(*itt);
                pindex[*itt] = i;
                i++;
            }
        }
    }
    else {
        plist = tmpplist;
    }

    psize = static_cast<int>(plist.size());
    pvals.resize(psize);
    pmap.clear();
    for (int j = 0; j < psize; j++) {
        pmap[plist[j]] = &pvals[j];
        pvals[j] = *plist[j];
    }
    for (MAP_pD_I::const_iterator itr = rindex.begin(); itr != rindex.end(); ++itr) {
        pmap[itr->first] = &pvals[itr->second];
    }

    c2p.clear();
    p2c.clear();
    for (std::vector<Constraint*>::iterator constr = clist.begin(); constr != clist.end(); ++constr) {
        (*constr)->revertParams();  // ensure that the constraint points to the original parameters
        VEC_pD constr_params_orig = (*constr)->params();
        SET_pD constr_params;
        for (VEC_pD::const_iterator p = constr_params_orig.begin(); p != constr_params_orig.end();
             ++p) {
            MAP_pD_pD::const_iterator pmapfind = pmap.find(*p);
            if (pmapfind != pmap.end()) {
                constr_params.insert(pmapfind->second);
            }
        }
        for (SET_pD::const_iterator p = constr_params.begin(); p != constr_params.end(); ++p) {
            //            jacobi.set(*constr, *p, 0.);
            c2p[*constr].push_back(*p);
            p2c[*p].push_back(*constr);
        }
        //        (*constr)->redirectParams(pmap); // redirect parameters to pvec
    }
}

void SubSystem::redirectParams()
{
    // copying values to pvals
    for (MAP_pD_pD::const_iterator p = pmap.begin(); p != pmap.end(); ++p) {
        *(p->second) = *(p->first);
    }

    // redirect constraints to point to pvals
    for (std::vector<Constraint*>::iterator constr = clist.begin(); constr != clist.end(); ++constr) {
        (*constr)->revertParams();  // this line will normally not be necessary
        (*constr)->redirectParams(pmap);
    }
}

void SubSystem::revertParams()
{
    for (std::vector<Constraint*>::iterator constr = clist.begin(); constr != clist.end(); ++constr) {
        (*constr)->revertParams();
    }
}

void SubSystem::getParamMap(MAP_pD_pD& pmapOut)
{
    pmapOut = pmap;
}

void SubSystem::getParamList(VEC_pD& plistOut)
{
    plistOut = plist;
}

void SubSystem::getParams(VEC_pD& params, Eigen::VectorXd& xOut)
{
    if (xOut.size() != int(params.size())) {
        xOut.setZero(params.size());
    }

    for (int j = 0; j < int(params.size()); j++) {
        MAP_pD_pD::const_iterator pmapfind = pmap.find(params[j]);
        if (pmapfind != pmap.end()) {
            xOut[j] = *(pmapfind->second);
        }
    }
}

void SubSystem::getParams(Eigen::VectorXd& xOut)
{
    if (xOut.size() != psize) {
        xOut.setZero(psize);
    }

    for (int i = 0; i < psize; i++) {
        xOut[i] = pvals[i];
    }
}

void SubSystem::setParams(VEC_pD& params, Eigen::VectorXd& xIn)
{
    assert(xIn.size() == int(params.size()));
    for (int j = 0; j < int(params.size()); j++) {
        MAP_pD_pD::const_iterator pmapfind = pmap.find(params[j]);
        if (pmapfind != pmap.end()) {
            *(pmapfind->second) = xIn[j];
        }
    }
}

void SubSystem::setParams(Eigen::VectorXd& xIn)
{
    assert(xIn.size() == psize);
    for (int i = 0; i < psize; i++) {
        pvals[i] = xIn[i];
    }
}

void SubSystem::getConstraintList(std::vector<Constraint*>& clist_)
{
    clist_ = clist;
}

double SubSystem::error()
{
    double err = 0.;
    for (std::vector<Constraint*>::const_iterator constr = clist.begin(); constr != clist.end();
         ++constr) {
        double tmp = (*constr)->error();
        err += tmp * tmp;
    }
    err *= 0.5;
    return err;
}

void SubSystem::calcResidual(Eigen::VectorXd& r)
{
    assert(r.size() == csize);

    int i = 0;
    for (std::vector<Constraint*>::const_iterator constr = clist.begin(); constr != clist.end();
         ++constr, i++) {
        r[i] = (*constr)->error();
    }
}

void SubSystem::calcResidual(Eigen::VectorXd& r, double& err)
{
    assert(r.size() == csize);

    int i = 0;
    err = 0.;
    for (std::vector<Constraint*>::const_iterator constr = clist.begin(); constr != clist.end();
         ++constr, i++) {
        r[i] = (*constr)->error();
        err += r[i] * r[i];
    }
    err *= 0.5;
}

void SubSystem::calcJacobi(VEC_pD& params, Eigen::MatrixXd& jacobi)
{
    jacobi.setZero(csize, params.size());

    // Build reverse-map: pvals pointer → column index in jacobi
    std::unordered_map<double*, int> param_col;
    param_col.reserve(params.size());
    for (int j = 0; j < int(params.size()); ++j) {
        MAP_pD_pD::const_iterator pmapfind = pmap.find(params[j]);
        if (pmapfind != pmap.end()) {
            param_col[pmapfind->second] = j;
        }
    }

    int n = csize;

    // ── 1) Batch-vectorized analytical Jacobian for P2PDistance constraints ──
    std::vector<int> p2p_indices;
    p2p_indices.reserve(n);
    for (int i = 0; i < n; ++i) {
        if (clist[i]->getTypeId() == P2PDistance) {
            p2p_indices.push_back(i);
        }
    }

    int num_p2p = static_cast<int>(p2p_indices.size());
    if (num_p2p > 0) {
        Eigen::VectorXd x1(num_p2p), y1(num_p2p), x2(num_p2p), y2(num_p2p);
        Eigen::VectorXd scaleVec(num_p2p);
        std::vector<int> col_p1x(num_p2p), col_p1y(num_p2p);
        std::vector<int> col_p2x(num_p2p), col_p2y(num_p2p);
        std::vector<int> col_dist(num_p2p);

        auto get_col = [&](double* pval_ptr) -> int {
            auto it = param_col.find(pval_ptr);
            return (it != param_col.end()) ? it->second : -1;
        };

        for (int k = 0; k < num_p2p; ++k) {
            int idx = p2p_indices[k];
            Constraint* constr = clist[idx];
            x1[k] = *(constr->getPvec()[0]);
            y1[k] = *(constr->getPvec()[1]);
            x2[k] = *(constr->getPvec()[2]);
            y2[k] = *(constr->getPvec()[3]);
            scaleVec[k] = constr->getScale();

            col_p1x[k] = get_col(constr->getPvec()[0]);
            col_p1y[k] = get_col(constr->getPvec()[1]);
            col_p2x[k] = get_col(constr->getPvec()[2]);
            col_p2y[k] = get_col(constr->getPvec()[3]);
            col_dist[k] = get_col(constr->getPvec()[4]);
        }

        // Vectorized analytical derivatives:
        //   f = scale * (||p1 - p2|| - dist)
        //   df/dp1 =  scale * (dx/d, dy/d)
        //   df/dp2 = -scale * (dx/d, dy/d)
        //   df/d(dist) = -scale
        Eigen::VectorXd dx = x1 - x2;
        Eigen::VectorXd dy = y1 - y2;
        Eigen::VectorXd d = (dx.array().square() + dy.array().square()).sqrt();
        Eigen::VectorXd inv_d = (d.array() > 1e-15).select(d.array().inverse(), 0.0);

        Eigen::VectorXd grad_p1x = scaleVec.array() * dx.array() * inv_d.array();
        Eigen::VectorXd grad_p1y = scaleVec.array() * dy.array() * inv_d.array();
        Eigen::VectorXd grad_p2x = -grad_p1x;
        Eigen::VectorXd grad_p2y = -grad_p1y;
        Eigen::VectorXd grad_dist = -scaleVec;

        for (int k = 0; k < num_p2p; ++k) {
            int row = p2p_indices[k];
            if (col_p1x[k] >= 0)  { jacobi(row, col_p1x[k]) = grad_p1x[k]; }
            if (col_p1y[k] >= 0)  { jacobi(row, col_p1y[k]) = grad_p1y[k]; }
            if (col_p2x[k] >= 0)  { jacobi(row, col_p2x[k]) = grad_p2x[k]; }
            if (col_p2y[k] >= 0)  { jacobi(row, col_p2y[k]) = grad_p2y[k]; }
            if (col_dist[k] >= 0) { jacobi(row, col_dist[k]) = grad_dist[k]; }
        }
    }

    // ── 2) Per-constraint grad() for all other constraint types ──
    for (int i = 0; i < n; ++i) {
        if (clist[i]->getTypeId() == P2PDistance) {
            continue;  // already handled in the batch path above
        }
        for (const auto& entry : param_col) {
            jacobi(i, entry.second) = clist[i]->grad(entry.first);
        }
    }
}

void SubSystem::calcJacobi(Eigen::MatrixXd& jacobi)
{
    calcJacobi(plist, jacobi);
}

void SubSystem::calcGrad(VEC_pD& params, Eigen::VectorXd& grad)
{
    assert(grad.size() == int(params.size()));

    grad.setZero();
    for (int j = 0; j < int(params.size()); j++) {
        MAP_pD_pD::const_iterator pmapfind = pmap.find(params[j]);
        if (pmapfind != pmap.end()) {
            std::vector<Constraint*> constrs = p2c[pmapfind->second];
            for (std::vector<Constraint*>::const_iterator constr = constrs.begin();
                 constr != constrs.end();
                 ++constr) {
                grad[j] += (*constr)->error() * (*constr)->grad(pmapfind->second);
            }
        }
    }
}

void SubSystem::calcGrad(Eigen::VectorXd& grad)
{
    calcGrad(plist, grad);
}

double SubSystem::maxStep(VEC_pD& params, Eigen::VectorXd& xdir)
{
    assert(xdir.size() == int(params.size()));

    MAP_pD_D dir;
    for (int j = 0; j < int(params.size()); j++) {
        MAP_pD_pD::const_iterator pmapfind = pmap.find(params[j]);
        if (pmapfind != pmap.end()) {
            dir[pmapfind->second] = xdir[j];
        }
    }

    double alpha = 1e10;
    for (std::vector<Constraint*>::iterator constr = clist.begin(); constr != clist.end(); ++constr) {
        alpha = (*constr)->maxStep(dir, alpha);
    }

    return alpha;
}

double SubSystem::maxStep(Eigen::VectorXd& xdir)
{
    return maxStep(plist, xdir);
}

void SubSystem::applySolution()
{
    for (MAP_pD_pD::const_iterator it = pmap.begin(); it != pmap.end(); ++it) {
        *(it->first) = *(it->second);
    }
}

void SubSystem::analyse(Eigen::MatrixXd& /*J*/, Eigen::MatrixXd& /*ker*/, Eigen::MatrixXd& /*img*/)
{}

void SubSystem::report()
{}

void SubSystem::printResidual()
{
    Eigen::VectorXd r(csize);
    int i = 0;
    double err = 0.;
    for (std::vector<Constraint*>::const_iterator constr = clist.begin(); constr != clist.end();
         ++constr, i++) {
        r[i] = (*constr)->error();
        err += r[i] * r[i];
    }
    err *= 0.5;
    std::cout << "Residual r = " << r.transpose() << std::endl;
    std::cout << "Residual err = " << err << std::endl;
}


}  // namespace GCS
