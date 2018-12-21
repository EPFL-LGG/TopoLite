//=============================================================================
//
//   Code framework for the lecture
//
//   "Digital 3D Geometry Processing"
//
//   Gaspard Zoss, Alexandru Ichim
//
//   Copyright (C) 2016 by Computer Graphics and Geometry Laboratory,
//         EPF Lausanne
//
//-----------------------------------------------------------------------------
#include "viewer.h"
Viewer::Viewer() : nanogui::Screen(Eigen::Vector2i(1024, 768), "DGP Viewer")
{

	window_ = new Window(this, "Controls");
	window_->setPosition(Vector2i(15, 15));
	window_->setLayout(new GroupLayout());

	PopupButton *popupBtn = new PopupButton(window_, "Open a mesh", ENTYPO_ICON_EXPORT);
	Popup *popup = popupBtn->popup();
	popup->setLayout(new GroupLayout());

	performLayout();
}
