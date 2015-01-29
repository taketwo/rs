/*
 * Software License Agreement (BSD License)
 *
 *  Point Cloud Library (PCL) - www.pointclouds.org
 *  Copyright (c) 2014-, Open Perception, Inc.
 *
 *  All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above
 *     copyright notice, this list of conditions and the following
 *     disclaimer in the documentation and/or other materials provided
 *     with the distribution.
 *   * Neither the name of the copyright holder(s) nor the names of its
 *     contributors may be used to endorse or promote products derived
 *     from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 *  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 *  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 *  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 *  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 *  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.
 *
 */

#include <boost/lexical_cast.hpp>

#include <pxcimage.h>
#include <pxccapture.h>
#include <pxcsensemanager.h>
#include <pxcprojection.h>

#include <pcl/common/io.h>
#include <pcl/common/time.h>

#include "real_sense_grabber.h"
#include "real_sense/real_sense_device_manager.h"
#include "buffers.h"
#include "io_exception.h"

using namespace pcl::io::real_sense;

pcl::RealSenseGrabber::RealSenseGrabber (const std::string& device_id)
: Grabber ()
, is_running_ (false)
, confidence_threshold_ (6)
, temporal_filtering_type_ (RealSense_None)
, depth_buffer_ (new pcl::io::SingleBuffer<unsigned short> (SIZE))
{
  if (device_id == "")
    device_ = RealSenseDeviceManager::getInstance ()->captureDevice ();
  else if (device_id[0] == '#')
    device_ = RealSenseDeviceManager::getInstance ()->captureDevice (boost::lexical_cast<int> (device_id.substr (1)) - 1);
  else
    device_ = RealSenseDeviceManager::getInstance ()->captureDevice (device_id);

  point_cloud_signal_ = createSignal<sig_cb_real_sense_point_cloud> ();
  point_cloud_rgba_signal_ = createSignal<sig_cb_real_sense_point_cloud_rgba> ();
}

pcl::RealSenseGrabber::~RealSenseGrabber () throw ()
{
  stop ();

  disconnect_all_slots<sig_cb_real_sense_point_cloud> ();
  disconnect_all_slots<sig_cb_real_sense_point_cloud_rgba> ();
}

void
pcl::RealSenseGrabber::start ()
{
  if (!is_running_)
  {
    need_xyz_ = num_slots<sig_cb_real_sense_point_cloud> () > 0;
    need_xyzrgba_ = num_slots<sig_cb_real_sense_point_cloud_rgba> () > 0;
    if (need_xyz_ || need_xyzrgba_)
    {
      frequency_.reset ();
      is_running_ = true;
      PXCCapture::Device::StreamProfileSet profile;
      memset (&profile, 0, sizeof (profile));
      // TODO: this should depend on Mode
      profile.depth.frameRate.max = 30;
      profile.depth.frameRate.min = 30;
      profile.depth.imageInfo.width = WIDTH;
      profile.depth.imageInfo.height = HEIGHT;
      profile.depth.imageInfo.format = PXCImage::PIXEL_FORMAT_DEPTH;
      profile.depth.options = PXCCapture::Device::STREAM_OPTION_ANY;
      if (need_xyzrgba_)
      {
        profile.color.frameRate.max = 30;
        profile.color.frameRate.min = 30;
        profile.color.imageInfo.width = COLOR_WIDTH;
        profile.color.imageInfo.height = COLOR_HEIGHT;
        profile.color.imageInfo.format = PXCImage::PIXEL_FORMAT_RGB32;
        profile.color.options = PXCCapture::Device::STREAM_OPTION_ANY;
      }
      device_->getPXCDevice ().SetStreamProfileSet (&profile);
      bool valid = device_->getPXCDevice ().IsStreamProfileSetValid (&profile);
      if (!valid)
        THROW_IO_EXCEPTION ("invalid stream profile for PXC device");

      thread_ = boost::thread (&RealSenseGrabber::run, this);
    }
  }
}

void
pcl::RealSenseGrabber::stop ()
{
  if (is_running_)
  {
    is_running_ = false;
    thread_.join ();
  }
}

bool
pcl::RealSenseGrabber::isRunning () const
{
  return (is_running_);
}

float
pcl::RealSenseGrabber::getFramesPerSecond () const
{
  boost::mutex::scoped_lock lock (fps_mutex_);
  return (frequency_.getFrequency ());
}

void
pcl::RealSenseGrabber::setConfidenceThreshold (unsigned int threshold)
{
  if (threshold > 15)
  {
    PCL_WARN ("[pcl::RealSenseGrabber::setConfidenceThreshold] Attempted to set threshold outside valid range (0-15)");
  }
  else
  {
    confidence_threshold_ = threshold;
    device_->getPXCDevice ().SetDepthConfidenceThreshold (confidence_threshold_);
  }
}

void
pcl::RealSenseGrabber::enableTemporalFiltering (TemporalFilteringType type, size_t window_size)
{
  if (temporal_filtering_type_ != type ||
     (type != RealSense_None && depth_buffer_->size () != window_size))
  {
    bool was_running = is_running_;
    if (was_running)
      stop ();
    switch (type)
    {
      case RealSense_None:
        {
          depth_buffer_.reset (new pcl::io::SingleBuffer<unsigned short> (SIZE));
          break;
        }
      case RealSense_Median:
        {
          // TODO: MedianFilter freezes on destructor, investigate
          depth_buffer_.reset (new pcl::io::MedianBuffer<unsigned short> (SIZE, window_size));
          break;
        }
      case RealSense_Average:
        {
          depth_buffer_.reset (new pcl::io::AverageBuffer<unsigned short> (SIZE, window_size));
          break;
        }
    }
    temporal_filtering_type_ = type;
    if (was_running)
      start ();
  }
}

void
pcl::RealSenseGrabber::disableTemporalFiltering ()
{
  enableTemporalFiltering (RealSense_None, 1);
}

const std::string&
pcl::RealSenseGrabber::getDeviceSerialNumber () const
{
  return (device_->getSerialNumber ());
}

void
pcl::RealSenseGrabber::run ()
{
  PXCProjection* projection = device_->getPXCDevice ().CreateProjection ();
  PXCCapture::Sample sample;
  std::vector<PXCPoint3DF32> vertices (SIZE);

  while (is_running_)
  {
    pcl::PointCloud<pcl::PointXYZ>::Ptr xyz_cloud;
    pcl::PointCloud<pcl::PointXYZRGBA>::Ptr xyzrgba_cloud;

    static const float nan = std::numeric_limits<float>::quiet_NaN ();

    pxcStatus status;
    if (need_xyzrgba_)
      status = device_->getPXCDevice ().ReadStreams (PXCCapture::STREAM_TYPE_DEPTH | PXCCapture::STREAM_TYPE_COLOR, &sample);
    else
      status = device_->getPXCDevice ().ReadStreams (PXCCapture::STREAM_TYPE_DEPTH, &sample);

    uint64_t timestamp = pcl::getTime () * 1.0e+6;
    
    switch (status)
    {
    case PXC_STATUS_NO_ERROR:
    {
      fps_mutex_.lock ();
      frequency_.event ();
      fps_mutex_.unlock ();

      /* We preform the following steps to convert received data into point clouds:
       * 
       *   1. Push depth image to the depth buffer
       *   2. Pull filtered depth image from the depth buffer
       *   3. Project (filtered) depth image into 3D
       *   4. Fill XYZ point cloud with computed points
       *   5. Fill XYZRGBA point cloud with computed points
       *   7. Project color image into 3D
       *   6. Assign colors to points in XYZRGBA point cloud
       *
       * Steps 1-2 are skipped if temporal filtering is disabled.
       * Step 4 is skipped if there are no subscribers for XYZ clouds.
       * Steps 5-7 are skipped if there are no subscribers for XYZRGBA clouds. */

      if (temporal_filtering_type_ != RealSense_None)
      {
        PXCImage::ImageData data;
        sample.depth->AcquireAccess (PXCImage::ACCESS_READ, &data);
        std::vector<unsigned short> data_copy (SIZE);
        memcpy (data_copy.data (), data.planes[0], SIZE * sizeof (unsigned short));
        sample.depth->ReleaseAccess (&data);

        depth_buffer_->push (data_copy);

        sample.depth->AcquireAccess (PXCImage::ACCESS_WRITE, &data);
        unsigned short* d = reinterpret_cast<unsigned short*> (data.planes[0]);
        for (size_t i = 0; i < SIZE; i++)
          d[i] = (*depth_buffer_)[i];
        sample.depth->ReleaseAccess (&data);
      }

      projection->QueryVertices (sample.depth, vertices.data ());

      if (need_xyz_)
      {
        xyz_cloud.reset (new pcl::PointCloud<pcl::PointXYZ> (WIDTH, HEIGHT));
        xyz_cloud->header.stamp = timestamp;
        xyz_cloud->is_dense = false;
        for (int i = 0; i < SIZE; i++)
        {
          if (vertices[i].z == 0)
            xyz_cloud->points[i].x = xyz_cloud->points[i].y = xyz_cloud->points[i].z = nan;
          else
          {
            xyz_cloud->points[i].x = vertices[i].x / 1000.0;
            xyz_cloud->points[i].y = vertices[i].y / 1000.0;
            xyz_cloud->points[i].z = vertices[i].z / 1000.0;
          }
        }
      }

      if (need_xyzrgba_)
      {
        PXCImage::ImageData data;
        PXCImage* mapped = projection->CreateColorImageMappedToDepth (sample.depth, sample.color);
        mapped->AcquireAccess (PXCImage::ACCESS_READ, &data);
        uint32_t* d = reinterpret_cast<uint32_t*> (data.planes[0]);
        if (need_xyz_)
        {
          // We can fill XYZ coordinates more efficiently using pcl::copyPointCloud,
          // given that they were already computed for XYZ point cloud.
          xyzrgba_cloud.reset (new pcl::PointCloud<pcl::PointXYZRGBA>);
          pcl::copyPointCloud (*xyz_cloud, *xyzrgba_cloud);
          for (int i = 0; i < SIZE; i++)
          {
            memcpy (&xyzrgba_cloud->points[i].rgba, &d[i], sizeof (uint32_t));
          }
        }
        else
        {
          xyzrgba_cloud.reset (new pcl::PointCloud<pcl::PointXYZRGBA> (WIDTH, HEIGHT));
          xyzrgba_cloud->header.stamp = timestamp;
          xyzrgba_cloud->is_dense = false;
          for (int i = 0; i < SIZE; i++)
          {
            if (vertices[i].z == 0)
              xyzrgba_cloud->points[i].x = xyzrgba_cloud->points[i].y = xyzrgba_cloud->points[i].z = nan;
            else
            {
              xyzrgba_cloud->points[i].x = vertices[i].x / 1000.0;
              xyzrgba_cloud->points[i].y = vertices[i].y / 1000.0;
              xyzrgba_cloud->points[i].z = vertices[i].z / 1000.0;
            }
            memcpy (&xyzrgba_cloud->points[i].rgba, &d[i], sizeof (uint32_t));
          }
        }
        mapped->ReleaseAccess (&data);
        mapped->Release ();
      }

      if (need_xyzrgba_)
        point_cloud_rgba_signal_->operator () (xyzrgba_cloud);
      if (need_xyz_)
        point_cloud_signal_->operator () (xyz_cloud);
      break;
    }
    case PXC_STATUS_DEVICE_LOST:
      THROW_IO_EXCEPTION ("failed to read data stream from PXC device: device lost");
    case PXC_STATUS_ALLOC_FAILED:
      THROW_IO_EXCEPTION ("failed to read data stream from PXC device: alloc failed");
    }
    sample.ReleaseImages ();
  }
  projection->Release ();
  // TODO: replace with a custom "restart" function?
  std::string id = device_->getSerialNumber ();
  device_.reset ();
  device_ = RealSenseDeviceManager::getInstance ()->captureDevice (id);
}
