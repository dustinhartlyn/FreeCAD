// SPDX-License-Identifier: LGPL-2.1-or-later

/***************************************************************************
 *   Copyright (c) 2011 Jürgen Riegel <juergen.riegel@web.de>              *
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

#pragma once

#include <QDialog>

#include <Base/Placement.h>
#include <Gui/Selection/Selection.h>
#include <Mod/Sketcher/SketcherGlobal.h>

#include <fastsignals/signal.h>


namespace SketcherGui
{

class Ui_SketchOrientationDialog;
class SketcherGuiExport SketchOrientationDialog: public QDialog
{
    Q_OBJECT

public:
    SketchOrientationDialog();
    ~SketchOrientationDialog() override;

    Base::Placement Pos;
    int DirType;

    void accept() override;
    void reject() override;

protected Q_SLOTS:
    void onPreview();

private Q_SLOTS:
    // Phase 5o: Auto-advance dialog when user selects a base plane in 3D view
    void onSelectionChanged(const Gui::SelectionChanges& msg);

private:
    std::unique_ptr<Ui_SketchOrientationDialog> ui;
    fastsignals::scoped_connection connectSelection;
};

}  // namespace SketcherGui
