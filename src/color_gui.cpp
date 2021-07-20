/*********************************************************************
 * Copyright (c) 2008, Willow Garage, Inc.
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 *     * Redistributions of source code must retain the above copyright
 *       notice, this list of conditions and the following disclaimer.
 *     * Redistributions in binary form must reproduce the above copyright
 *       notice, this list of conditions and the following disclaimer in the
 *       documentation and/or other materials provided with the distribution.
 *     * Neither the name of Willow Garage, Inc. nor the names of its
 *       contributors may be used to endorse or promote products derived from
 *       this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *********************************************************************/

/** \author Nate Koenig and Peter Pastor */

#include <string>
#include <cv_bridge/cv_bridge.h>

#include "color_gui.h"
#include "conversions.h"

bool ColorGuiApp::OnInit()
{
  char **local_argv = new char*[ argc ];
  for (int i =0; i < argc; i++)
    local_argv[i] = strdup( wxString( argv[i] ).mb_str() );

  ros::init(argc, local_argv, "cmvision");
  ros::NodeHandle node_handle;

  // Subscribe to an image stream
  image_subscriber_ = node_handle.subscribe("image", 1, &ColorGuiApp::imageCB, this);

  frame_ = new ColorGuiFrame();
  frame_->Show(true);
  SetTopWindow(frame_);

  update_timer_ = new wxTimer(this);
  update_timer_->Start( 33 );

  Connect( update_timer_->GetId(), wxEVT_TIMER, wxTimerEventHandler( ColorGuiApp::OnUpdate ), NULL, this);

  return true;
}

void ColorGuiApp::OnUpdate( wxTimerEvent &event )
{
  ros::spinOnce();
}

void ColorGuiApp::imageCB(const sensor_msgs::ImageConstPtr& msg)
{
  frame_->DrawImage( msg );
}




ColorGuiFrame::ColorGuiFrame()
  : wxFrame(NULL, -1, wxT("Color Gui"), wxDefaultPosition, wxSize(800,600), wxDEFAULT_FRAME_STYLE)
{

  wxInitAllImageHandlers();

  wxMenuBar *menuBar = new wxMenuBar;
  wxMenu *file_menu = new wxMenu;

  wxMenuItem *item = file_menu->Append(ID_Reset, wxT("&Reset\tCtrl-R"));
  Connect(item->GetId(), wxEVT_COMMAND_MENU_SELECTED, wxCommandEventHandler(ColorGuiFrame::OnReset), NULL, this);

  item = file_menu->Append(wxID_EXIT, wxT("&Quit\tCtrl-Q"));
  Connect(item->GetId(), wxEVT_COMMAND_MENU_SELECTED, wxCommandEventHandler(ColorGuiFrame::OnQuit), NULL, this);

  menuBar->Append( file_menu, _("&File") );
  
  SetMenuBar( menuBar );

  CreateStatusBar();
  SetStatusText( _("Click on the image to select colors. Use scroll wheel to zoom.") );

  image_panel_ = new wxPanel(this, wxID_ANY, wxPoint(0,0), wxSize(640,480));

  wxStaticText *rgblabel = new wxStaticText(this, -1, wxT("RGB:"));
  rgbText_ = new wxTextCtrl(this,-1,wxT(""));

  wxStaticText *abLabel = new wxStaticText(this, -1, wxT("AB:"));
  abText_ = new wxTextCtrl(this,-1,wxT(""));

  wxBoxSizer *hsizer1 = new wxBoxSizer(wxHORIZONTAL);
  wxBoxSizer *hsizer2 = new wxBoxSizer(wxHORIZONTAL);
  wxBoxSizer *vsizer = new wxBoxSizer(wxVERTICAL);

  hsizer2->Add( rgblabel, 0, wxALIGN_CENTER_VERTICAL| wxLEFT, 10);
  hsizer2->Add( rgbText_, 1, wxALIGN_CENTER_VERTICAL | wxLEFT, 2);

  hsizer2->Add( abLabel, 0, wxALIGN_CENTER_VERTICAL | wxLEFT, 10);
  hsizer2->Add( abText_, 1, wxALIGN_CENTER_VERTICAL | wxLEFT, 2);
  hsizer2->Add(50,1,0);

  vsizer->Add(image_panel_, 0, wxEXPAND);
  vsizer->Add(20,2,0);
  vsizer->Add(hsizer2, 0, wxALIGN_LEFT | wxEXPAND );
  this->SetSizer(vsizer);

	width_ = 0;
	height_ = 0;

  scale_ = 1.0;
  width_ = 0;
  height_ = 0;

  rgb_image_ = NULL;
  lab_image_ = NULL;

  vision_ = new CMVision();

  image_panel_->Connect(wxEVT_LEFT_UP, wxMouseEventHandler(ColorGuiFrame::OnClick), NULL, this);

  Connect(wxEVT_MOUSEWHEEL, wxMouseEventHandler(ColorGuiFrame::OnMouseWheel));

}

void ColorGuiFrame::OnQuit(wxCommandEvent &event)
{
  Close(true);
}

void ColorGuiFrame::DrawImage(const sensor_msgs::ImageConstPtr& msg)
{
  IplImage cvImageRef, *cvImage;
  CvSize size;

  const sensor_msgs::Image img = *msg;

  // Get the image as and RGB image
  cv_bridge::CvImagePtr image_ptr = cv_bridge::toCvCopy(msg, "rgb8");
  cvImageRef = IplImage(image_ptr->image);
  cvImage = &cvImageRef;

  size = cvGetSize(cvImage);

  if (width_ != size.width || height_ != size.height)
  {
    if (!rgb_image_)
      delete[] rgb_image_;
    rgb_image_ = new unsigned char[size.width * size.height * 3];

    if (!lab_image_)
      delete[] lab_image_;
    lab_image_ = new unsigned char[size.width * size.height * 3];

    if (!(vision_->initialize(size.width, size.height)))
    {
      width_ = height_ = 0;
      ROS_ERROR("Vision init failed.");
      return;
    }
  }

  width_ = size.width;
  height_ = size.height;

  memcpy(rgb_image_, cvImage->imageData, width_ * height_ * 3);

  // Convert image to LAB color space
  cvCvtColor(cvImage, cvImage, CV_RGB2Lab);
  
  memcpy(lab_image_, cvImage->imageData, width_ * height_ * 3);

  // Find the color blobs
  if (!vision_->processFrame(reinterpret_cast<image_pixel*> (cvImage->imageData)))
  {
	ROS_ERROR("Frame error.");
	return;
  }

  int xsrc = (scale_pos_x_*scale_) - scale_pos_x_;
  int ysrc = (scale_pos_y_*scale_) - scale_pos_y_;

  wxImage image(width_, height_, rgb_image_, true);
  image.Rescale(width_*scale_,height_*scale_);

  wxBitmap bitmap(image);

  wxMemoryDC memDC;
  memDC.SelectObject(bitmap);

  wxClientDC dc(image_panel_);
  if (xsrc < 0 || ysrc < 0)
    dc.Clear();

  dc.Blit(0,0, 640, 480, &memDC, xsrc, ysrc);

	// Get all the blobs
	for (int ch = 0; ch < CMV_MAX_COLORS; ++ch)
	{
		// Get the descriptive color
		rgb c = vision_->getColorVisual(ch);

		// Grab the regions for this color
		CMVision::region* r = NULL;

		for (r = vision_->getRegions(ch); r != NULL; r = r->next)
		{
      dc.SetBrush(*wxTRANSPARENT_BRUSH);
      int x1 = (r->x1*scale_) - xsrc;
      int y1 = (r->y1*scale_) - ysrc;
      int x2 = (r->x2*scale_) - xsrc;
      int y2 = (r->y2*scale_) - ysrc;

      int w = x2 - x1;
      int h = y2 - y1;

      dc.DrawRectangle(x1, y1, w, h);
		}
	}

  int x, y;
  GetPosition(&x, &y);
// Setting size is commented out because it breaks the gui:
// Text boxes become invisible.
//  SetSize(x,y, width_, height_+80);
}


void ColorGuiFrame::OnReset(wxCommandEvent &event)
{
  vision_->setThreshold(0, 0, 0, 0, 0);
  rgbText_->SetValue(wxString::FromAscii(""));
  abText_->SetValue(wxString::FromAscii(""));
}

void ColorGuiFrame::OnClick(wxMouseEvent &event)
{
  int r, g, b, l, a, b2;

  int px = (event.m_x/scale_) + ((scale_pos_x_*scale_) - scale_pos_x_)/scale_;
  int py = (event.m_y/scale_) + ((scale_pos_y_*scale_) - scale_pos_y_)/scale_;

  r = rgb_image_[py * (width_ * 3) + px * 3 + 0];
  g = rgb_image_[py * (width_ * 3) + px * 3 + 1];
  b = rgb_image_[py * (width_ * 3) + px * 3 + 2];

  std::ostringstream stream1;
  stream1 << "(" <<  r  << ", " << g << ", " << b << ")";
  rgbText_->SetValue(wxString::FromAscii(stream1.str().c_str()));

  l = lab_image_[py * (width_ * 3) + px * 3 + 0];
  a = lab_image_[py * (width_ * 3) + px * 3 + 1];
  b2 = lab_image_[py * (width_ * 3) + px * 3 + 2];


  int l_low, l_high, a_low, a_high, b_low, b_high;

  vision_->getThreshold(0, a_low, a_high, b_low, b_high);

  if (a_low == 0 && a_high == 0)
  {
    a_low = a;
    a_high = a;
  }
  if (b_low == 0 && b_high == 0)
  {
    b_low = b2;
    b_high = b2;
  }

  a_low = std::min(a, a_low);
  a_high = std::max(a, a_high);

  b_low = std::min(b2, b_low);
  b_high = std::max(b2, b_high);

  vision_->setThreshold(0, a_low, a_high, b_low, b_high);

  std::ostringstream stream;
  stream << "( " << a_low << ":" << a_high << ", "
         << b_low << ":" << b_high << " ) ";

  abText_->SetValue(wxString::FromAscii(stream.str().c_str()));
}

void ColorGuiFrame::OnMouseWheel(wxMouseEvent &event)
{
  if (event.GetWheelRotation() < 0)
    scale_ *= 0.9;
  else
    scale_ *= 1.1;

  scale_pos_x_ = event.m_x;
  scale_pos_y_ = event.m_y;
}
